const form = document.querySelector('#python-run');
const status = document.querySelector('#python-status');
const output = document.querySelector('#python-output');
const cancel = document.querySelector('#cancel-run');
const run = form.querySelector('[type=submit]');
let worker;
let csv;

function reset() {
  run.disabled = false;
  cancel.hidden = true;
}

function drawPlot(canvas, pairs, equalScale = false) {
  const context = canvas.getContext('2d');
  const [width, height] = [canvas.width, canvas.height];
  const [left, right, top, bottom] = [67, width - 18, 20, height - 38];
  let [xmin, xmax, ymin, ymax] = [
    Math.min(...pairs.map(point => point[0])),
    Math.max(...pairs.map(point => point[0])),
    Math.min(...pairs.map(point => point[1])),
    Math.max(...pairs.map(point => point[1]))
  ];
  const xspan = Math.max(xmax - xmin, .001);
  const yspan = Math.max(ymax - ymin, .001);
  const xmid = (xmax + xmin) / 2;
  const ymid = (ymax + ymin) / 2;
  const ratio = (right - left) / (bottom - top);
  xmin = xmid - 0.53 * (equalScale ? Math.max(xspan, yspan * ratio) : xspan);
  xmax = xmid + (xmid - xmin);
  ymin = ymid - 0.53 * (equalScale ? Math.max(yspan, xspan / ratio) : yspan);
  ymax = ymid + (ymid - ymin);
  context.fillStyle = '#171a19';
  context.fillRect(0, 0, width, height);
  context.font = '15px monospace';
  context.fillStyle = '#c0c5bc';
  context.strokeStyle = '#3c413b';
  context.beginPath();
  for (let division = 0; division <= 4; division++) {
    const fraction = division / 4;
    const x = left + (right - left) * fraction;
    const y = bottom - (bottom - top) * fraction;
    context.moveTo(x, top);
    context.lineTo(x, bottom);
    context.moveTo(left, y);
    context.lineTo(right, y);
    context.textAlign = 'center';
    context.fillText((xmin + (xmax - xmin) * fraction).toFixed(2), x, height - 12);
    context.textAlign = 'right';
    context.fillText((ymin + (ymax - ymin) * fraction).toFixed(2), left - 7, y + 5);
  }
  context.stroke();
  context.beginPath();
  pairs.forEach(([x, y], index) => {
    const px = left + (x - xmin) / (xmax - xmin) * (right - left);
    const py = bottom - (y - ymin) / (ymax - ymin) * (bottom - top);
    if (index === 0) context.moveTo(px, py);
    else context.lineTo(px, py);
  });
  context.lineWidth = 2.5;
  context.strokeStyle = '#9ed6ff';
  context.stroke();
  context.lineWidth = 1;
}

function showRun(result) {
  const {samples, duration, peakTorque} = result;
  const last = samples.at(-1);
  const metrics = [
    ['Samples', String(samples.length)],
    ['Run time', duration.toFixed(2) + ' s'],
    ['Final axle X / Y', last[1].toFixed(3) + ' / ' + last[2].toFixed(3) + ' m'],
    ['Peak axle torque', peakTorque.toFixed(3) + ' N·m'],
  ];
  const list = document.querySelector('#python-metrics');
  list.replaceChildren(...metrics.map(([label, value]) => {
    const item = document.createElement('div');
    const term = document.createElement('dt');
    const number = document.createElement('dd');
    term.textContent = label;
    number.textContent = value;
    item.append(term, number);
    return item;
  }));
  drawPlot(document.querySelector('#plot-path'), samples.map(row => row.slice(1, 3)), true);
  drawPlot(document.querySelector('#plot-caster'), samples.map(([t, , , angle]) => [t, angle]));
  csv = result.csv;
  output.hidden = false;
  status.textContent = 'Simulation complete. Results were generated in this browser.';
}

function updateFields() {
  for (const id of ['maneuver-field','profile-field']) {
    const field = document.getElementById(id);
    field.hidden = form.elements.model.value === 'V1';
    field.querySelector('select').disabled = field.hidden;
  }
}
form.elements.model.addEventListener('change', updateFields);
updateFields();
reset();
form.addEventListener('submit', event => {
  event.preventDefault();
  const settings = Object.fromEntries(new FormData(form));
  settings.caster = Number(settings.caster);
  status.textContent = worker ? 'Integrating the simulation…' : 'Loading Python and NumPy…';
  run.disabled = true;
  cancel.hidden = false;
  if (!worker) {
    try {
      worker = new Worker(new URL('./python-worker.mjs', import.meta.url), {type: 'module'});
    } catch {
      status.textContent = 'This browser could not start Python. See the local setup below.';
      reset();
      return;
    }
    worker.addEventListener('message', ({data}) => {
      if (data.status) status.textContent = data.status;
      else {
        if (data.result) showRun(data.result);
        else status.textContent = 'Simulation could not run. ' + data.error.split('\n').filter(Boolean).at(-1);
        reset();
      }
    });
    worker.addEventListener('error', () => {
      worker.terminate();
      worker = null;
      status.textContent = 'The Python runtime could not load. Retry or use the local setup below.';
      reset();
    });
  }
  worker.postMessage(settings);
});
cancel.addEventListener('click', () => {
  worker?.terminate();
  worker = null;
  status.textContent = 'Simulation stopped.';
  reset();
});
document.querySelector('#download-run').addEventListener('click', () => {
  const url = URL.createObjectURL(new Blob([csv], {type: 'text/csv'}));
  const link = document.createElement('a');
  link.href = url;
  link.download = 'arche-browser-run.csv';
  link.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
});
