#ifndef WEBPAGE_H
#define WEBPAGE_H
const char index_html[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html>
<body style="background:#050505; color:#0f0; font-family:monospace; padding:20px;">
    <style>
        input { background:#1a1a1a; color:#0f0; border:1px solid #333; padding:5px; width:100px; }
        button { background:#333; color:#0f0; border:1px solid #555; padding:5px 10px; cursor:pointer; }
    </style>
    <div>Freq (MHz): <input id="f" value="915.000"> <button onclick="fetch('/ctrl?f='+document.getElementById('f').value)">Set Freq</button></div>
    <div style="margin:10px 0;">RSSI Gate: <input id="r" value="-85"> <button onclick="fetch('/ctrl?rssi='+document.getElementById('r').value)">Set Gate</button></div>
    <button onclick="fetch('/ctrl?force=1')" style="background:#300; color:#f00; border:1px solid #f00;">FORCE CAPTURE</button>
    <button onclick="fetch('/ctrl?clear=1')">CLEAR</button>
    <div id="status" style="margin-top:10px;">RSSI: -- dBm</div>
    <canvas id="c" width="1000" height="200" style="border:1px solid #333; margin-top:10px; display:block; background:#000;"></canvas>
    <script>
        async function poll() {
            try {
                let res = await fetch('/data'); let j = await res.json();
                document.getElementById('status').innerText = "RSSI: " + j.currentRSSI + " dBm";
                let ctx = document.getElementById('c').getContext('2d');
                ctx.clearRect(0,0,1000,200); ctx.strokeStyle='#0f0'; ctx.lineWidth=2; ctx.beginPath();
                j.data.forEach((v,i) => ctx.lineTo(i/2, v?50:150));
                ctx.stroke();
            } catch(e) {}
            setTimeout(poll, 500);
        } poll();
    </script>
</body>
</html>
)rawhtml";
#endif
