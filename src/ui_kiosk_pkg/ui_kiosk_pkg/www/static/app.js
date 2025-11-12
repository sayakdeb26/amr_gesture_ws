const el = id => document.getElementById(id);
let st=null, last_session=null;

function toast(msg){ const t=el('toast'); t.textContent=msg; t.style.display='block'; setTimeout(()=>t.style.display='none',1200); }
function setVideo(src){
  const v=el('clip'), idle=el('idle');
  if(src){ v.src=src; v.style.display='block'; idle.style.display='none'; }
  else { v.removeAttribute('src'); v.style.display='none'; idle.style.display='grid'; }
}
function renderHist(h){
  const box=document.getElementById('hist'); box.innerHTML='';
  (h||[]).forEach(it=>{
    const d=document.createElement('div');
    d.className='status';
    d.textContent = `${it.outcome} · ${it.final_label||it.candidate_label||''} · ${new Date(it.ts).toLocaleTimeString()}`;
    box.appendChild(d);
  });
}

// toggle fullscreen
document.addEventListener("DOMContentLoaded", () => {
  const fsBtn = document.getElementById("fullscreen-btn");
  if (!fsBtn) return;

  fsBtn.addEventListener("click", () => {
    if (!document.fullscreenElement) {
      document.documentElement.requestFullscreen().catch(err => {
        console.warn("Fullscreen request failed:", err);
      });
    } else {
      document.exitFullscreen();
    }
  });
});

async function pull(){
  try{
    const r=await fetch('/state'); if(!r.ok) throw 0;
    const s=await r.json(); st=s;
    el('label').value=s.label||''; el('conf').textContent=`conf ${(s.confidence??0).toFixed(2)}`;
    el('hint').textContent=s.hint||''; el('auto').checked=!!s.auto_approve;
    setVideo(s.media_src||null);
    if(s.time_left_ms!==undefined && s.timeout_ms){
      const pct=Math.max(0,Math.min(100,100*(s.time_left_ms/s.timeout_ms)));
      document.getElementById('fill').style.width=pct+'%';
      document.getElementById('status').textContent=s.media_src?`time left: ${(s.time_left_ms/1000).toFixed(1)}s`:'';
    }
    if(last_session!==s.session_id){ last_session=s.session_id; el('label').focus(); }
    renderHist(s.history||[]);
  }catch(e){/* ignore */}
  setTimeout(pull,1000);
}
async function send(approved){
  if(!st || !st.session_id) return;
  const body={ session_id:st.session_id, approved:approved, final_label: el('label').value||'' };
  const r=await fetch('/confirm',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
  if(r.ok) toast(approved?'Approved':'Rejected');
}
document.getElementById('approve').onclick=()=>send(true);
document.getElementById('reject').onclick =()=>send(false);
pull();
