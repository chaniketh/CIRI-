const $ = id => document.getElementById(id);
const token = document.querySelector('meta[name="rig-token"]').content;
let state = null, pending = false, connected = false, previousPhase = '', lastHistory = 0;
const insertionLabels = {on_surface:'On surface',in_surface:'In surface',deep_into_surface:'Deep into surface'};
const phaseLabels = {idle:'IDLE',ready:'READY',taring:'TARING',starting:'STARTING',running:'RUNNING',stopped:'STOPPED',error:'CHECK RIG'};
function error(message) { $('error').textContent = message; $('error').hidden = !message; }
function controls() {
  const phase = state?.phase;
  $('tare').disabled = pending || !connected || !['idle','ready','error'].includes(phase);
  $('start').disabled = pending || !connected || phase !== 'ready' || !state?.tared;
  $('finish').disabled = pending || !connected || !['running','stopped'].includes(phase);
  const speedAllowed = !pending && connected && !['taring','starting','stopped'].includes(phase);
  $('slower').disabled = !speedAllowed || state?.rpm <= 0.229;
  $('faster').disabled = !speedAllowed || state?.rpm >= 15.114;
  for (const id of ['material','angle','insertion','trial']) $(id).disabled = ['running','starting','stopped','taring'].includes(phase);
}
async function action(name, values={}) {
  pending = true; controls(); error('');
  try {
    const response = await fetch(`/api/${name}`, {method:'POST', headers:{'Content-Type':'application/json','X-Rig-Token':token},body:JSON.stringify(values)});
    const result = await response.json();
    if (!response.ok) throw new Error(result.error || 'Action failed');
    // Worker owns USB; leave time for it to take the command before polling.
    await new Promise(resolve => setTimeout(resolve,120));
    await poll();
  } catch (exc) { error(exc.message); }
  finally { pending = false; controls(); }
}
$('condition').addEventListener('submit', event => {
  event.preventDefault();
  if (!$('condition').reportValidity()) return;
  action('start', {material:$('material').value,needle_angle_deg:Number($('angle').value),
    engagement_level:$('insertion').value,trial_id:$('trial').value,notes:$('notes').value});
});
$('tare').addEventListener('click',()=>action('tare'));
$('finish').addEventListener('click',()=>action('finish',{notes:$('notes').value,outcome:$('outcome').value}));
$('slower').addEventListener('click',()=>action('speed',{rpm:Math.max(0.229,state.rpm-0.5)}));
$('faster').addEventListener('click',()=>action('speed',{rpm:Math.min(15.114,state.rpm+0.5)}));
function chart(points) {
  const canvas = $('chart'), ratio = window.devicePixelRatio || 1;
  const width = canvas.clientWidth, height = canvas.clientHeight;
  canvas.width = width*ratio; canvas.height = height*ratio;
  const ctx = canvas.getContext('2d'); ctx.scale(ratio,ratio);
  const left=40,right=8,top=10,bottom=20,w=width-left-right,h=height-top-bottom;
  let span=Math.max(0.05,...points.map(p=>Math.abs(p[1])))*1.15;
  const duration=Math.max(1,points.at(-1)?.[0] || 1);
  ctx.font='9px ui-monospace, monospace'; ctx.textAlign='right';
  for (const v of [-span,0,span]) {
    const y=top+h/2-v/span*h/2;
    ctx.strokeStyle=v===0?'#c1d0c4':'#e8eee7';ctx.beginPath();ctx.moveTo(left,y);ctx.lineTo(width-right,y);ctx.stroke();
    ctx.fillStyle='#879a8e';ctx.fillText(v.toFixed(2),left-8,y+3);
  }
  ctx.fillStyle='#879a8e';ctx.textAlign='left';ctx.fillText('0',left,height-3);
  ctx.textAlign='right';ctx.fillText(duration.toFixed(1),width-right,height-3);
  if (points.length>1) {
    ctx.strokeStyle='#236855';ctx.lineWidth=2;ctx.beginPath();
    points.forEach(([t,f],i)=>{const x=left+t/duration*w,y=top+h/2-f/span*h/2;i?ctx.lineTo(x,y):ctx.moveTo(x,y);});ctx.stroke();
  } else {
    ctx.fillStyle='#a0afa4';ctx.textAlign='center';ctx.fillText('Force trace appears when a test starts',left+w/2,top+h/2-12);
  }
}
function cell(text, secondary='') {
  const td=document.createElement('td');td.textContent=text ?? '—';
  if(secondary){const small=document.createElement('small');small.textContent=secondary;td.append(small);}return td;
}
async function history() {
  const response=await fetch('/api/runs'); if(!response.ok)return;
  const rows=await response.json();$('log-count').textContent=`${rows.length} record${rows.length===1?'':'s'}`;
  const body=$('runs');body.replaceChildren();
  if(!rows.length){const row=document.createElement('tr');const td=cell('Your completed tests will appear here.');td.colSpan=6;td.className='empty';row.append(td);body.append(row);return;}
  for(const run of rows){
    const row=document.createElement('tr');row.append(cell(run.trial || run.id,`${run.demo?'DEMO · ':''}${run.material}`),
      cell(run.angle==null?'—':`${run.angle}°`),cell(insertionLabels[run.insertion] || '—'),
      cell(run.peak==null?'—':`${run.peak.toFixed(3)} N`),
      cell((run.outcome || run.status || '').replaceAll('_',' '),(run.reason || '').replaceAll('_',' ')));
    const td=document.createElement('td'),link=document.createElement('a');link.href=`/api/download/${encodeURIComponent(run.id)}`;link.textContent='Download ↗';td.append(link);row.append(td);body.append(row);
  }
}
async function poll() {
  try {
    const response=await fetch('/api/state',{signal:AbortSignal.timeout(7000)});
    if(!response.ok)throw new Error('Server unavailable');
    state=await response.json();connected=true;
    $('connection').textContent=state.demo?'Demo · simulated rig':state.tared?`Sensor connected · ${state.uno_port || ''}`:state.uno_port?`Uno detected · ${state.uno_port}`:(state.uno_port_error || 'Looking for Uno…');
    $('dot').classList.toggle('active',state.tared);
    $('force').textContent=state.force_N==null?'—':state.force_N.toFixed(3);
    $('peak').textContent=`${state.peak_N.toFixed(3)} N`;
    $('elapsed').textContent=`${state.elapsed_s.toFixed(1)} s`;
    $('speed').textContent=`${state.rpm.toFixed(3)} rpm`;
    $('actual').textContent=state.actual_rpm==null?'— rpm':`${Math.abs(state.actual_rpm).toFixed(3)} rpm`;
    $('phase').textContent=phaseLabels[state.phase] || state.phase.toUpperCase();
    $('phase').classList.toggle('running',state.phase==='running');
    $('message').textContent=state.message;
    $('limits').textContent=`${state.limits.max_abs_sensor_force_N == null ? 'No force stop' : state.limits.max_abs_sensor_force_N + ' N'} · ${state.limits.max_motor_path_deg == null ? 'No travel stop' : state.limits.max_motor_path_deg + '°'} · ${state.limits.duration_s} s`;
    $('demo-label').textContent=state.demo?' / SIMULATED DEMO':'';
    chart(state.trace);
    if(previousPhase!==state.phase || Date.now()-lastHistory>3000){await history();lastHistory=Date.now();}
    previousPhase=state.phase;
  } catch(exc){connected=false;$('connection').textContent='Server connection lost';$('dot').classList.remove('active');error('Connection lost. Active tests stop if the page cannot reach the server.');}
  controls();
}
window.addEventListener('resize',()=>chart(state?.trace || []));
async function cycle(){await poll();setTimeout(cycle,250);}cycle();
