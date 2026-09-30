import {getDocument, GlobalWorkerOptions, TextLayer} from './pdfjs/pdf.min.mjs';
GlobalWorkerOptions.workerSrc = new URL('./pdfjs/pdf.worker.min.mjs', import.meta.url).href;
const stage = document.querySelector('.pdf-stage');
const status = document.querySelector('#pdf-status');
const input = document.querySelector('#pdf-page');
const prev = document.querySelector('#pdf-prev'), next = document.querySelector('#pdf-next');
const fit = document.querySelector('#pdf-fit'), focus = document.querySelector('#pdf-focus');
const controls = [...document.querySelectorAll('.pdf-controls button, .pdf-controls input, .pdf-controls select')];
let pdf, pages = [], sheets = [], current = 1, generation = 0, observer;
const jobs = new Set();
function updateCurrent(number) {
  current = number; input.value = number;
  prev.disabled = number === 1; next.disabled = number === pdf.numPages;
  status.textContent = `Page ${number} of ${pdf.numPages}`;
}
function go(number) {
  if (!pdf || !Number.isInteger(number) || number < 1 || number > pdf.numPages) {input.value = current; return;}
  stage.scrollTop = sheets[number - 1].offsetTop - sheets[0].offsetTop;
  updateCurrent(number);
}
async function paint(sheet, page, viewport, version) {
  if (sheet.dataset.started) return;
  sheet.dataset.started = 'true';
  const canvas = document.createElement('canvas'); canvas.setAttribute('aria-hidden', 'true');
  if (page.pageNumber === 1) canvas.id = 'pdf-canvas';
  const ratio = Math.min(devicePixelRatio || 1, 2);
  canvas.width = Math.ceil(viewport.width * ratio); canvas.height = Math.ceil(viewport.height * ratio);
  canvas.style.width = `${viewport.width}px`; canvas.style.height = `${viewport.height}px`;
  sheet.append(canvas);
  try {
    const job = page.render({canvas, canvasContext:canvas.getContext('2d'), viewport, transform:[ratio,0,0,ratio,0,0]});
    jobs.add(job);
    try {await job.promise;} finally {jobs.delete(job);}
    if (version !== generation) return;
    const text = await page.getTextContent();
    if (version !== generation) return;
    const layer = document.createElement('div'); layer.className = 'textLayer';
    sheet.append(layer);
    await new TextLayer({textContentSource:text, container:layer, viewport}).render();
    sheet.dataset.ready = 'true';
  } catch (error) {
    if (version !== generation || error.name === 'RenderingCancelledException') return;
    sheet.dataset.loading = 'Page unavailable. Download the original PDF above.';
    status.textContent = `Page ${page.pageNumber} could not be displayed.`;
  }
}
function layout() {
  if (!pdf) return;
  const version = ++generation;
  observer?.disconnect();
  for (const job of jobs) job.cancel();
  // Release old canvas buffers when changing scale, especially on mobile.
  for (const canvas of stage.querySelectorAll('canvas')) {canvas.width = 0; canvas.height = 0;}
  stage.replaceChildren(); sheets = [];
  const padding = innerWidth <= 760 ? 24 : 48;
  for (const page of pages) {
    const base = page.getViewport({scale:1});
    const widthScale = Math.min(stage.clientWidth - padding, 820) / base.width;
    const scale = fit.value === 'width' ? widthScale : fit.value === 'page' ? Math.min(widthScale, (stage.clientHeight - padding) / base.height) : Number(fit.value);
    const viewport = page.getViewport({scale});
    const sheet = document.createElement('div'); sheet.className = 'pdf-sheet';
    sheet.dataset.page = page.pageNumber; sheet.dataset.loading = `Loading page ${page.pageNumber}…`;
    sheet.setAttribute('role', 'region'); sheet.setAttribute('aria-label', `Page ${page.pageNumber}`);
    sheet.style.width = `${viewport.width}px`; sheet.style.height = `${viewport.height}px`;
    sheet.style.setProperty('--total-scale-factor', scale * (page.userUnit || 1));
    sheet.style.setProperty('--scale-round-x', '1px'); sheet.style.setProperty('--scale-round-y', '1px');
    stage.append(sheet); sheets.push(sheet);
  }
  observer = new IntersectionObserver(entries => {
    for (const entry of entries) if (entry.isIntersecting) {
      const index = Number(entry.target.dataset.page) - 1;
      const base = pages[index].getViewport({scale:1});
      paint(entry.target, pages[index], pages[index].getViewport({scale:entry.target.clientWidth / base.width}), version);
      observer.unobserve(entry.target);
    }
  }, {root:stage, rootMargin:'500px'});
  sheets.forEach(sheet => observer.observe(sheet));
  go(current);
}
prev.addEventListener('click', () => go(current - 1));
next.addEventListener('click', () => go(current + 1));
input.addEventListener('change', () => go(Number(input.value)));
fit.addEventListener('change', layout);
function zoom(direction) {
  const base = pages[current - 1].getViewport({scale:1});
  const scale = sheets[current - 1].clientWidth / base.width;
  const levels = [0.5, 0.75, 1, 1.25, 1.5, 2];
  const value = direction > 0 ? levels.find(value => value > scale + 0.01) || 2 : [...levels].reverse().find(value => value < scale - 0.01) || 0.5;
  if (!fit.querySelector(`option[value="${value}"]`)) fit.add(new Option(`${value * 100}%`, String(value)));
  fit.value = String(value); layout();
}
document.querySelector('#pdf-out').addEventListener('click', () => zoom(-1));
document.querySelector('#pdf-in').addEventListener('click', () => zoom(1));
focus.addEventListener('click', () => {
  const active = document.body.classList.toggle('reader-focus');
  focus.setAttribute('aria-pressed', String(active)); focus.textContent = active ? 'Exit focus' : 'Focus'; focus.setAttribute('aria-label', active ? 'Exit focus' : 'Focus view'); layout();
});
document.addEventListener('keydown', event => {
  if (event.key === 'Escape' && document.body.classList.contains('reader-focus')) focus.click();
});
let scrolling;
stage.addEventListener('scroll', () => {
  cancelAnimationFrame(scrolling);
  scrolling = requestAnimationFrame(() => {
    if (!sheets.length) return;
    const line = stage.getBoundingClientRect().top + stage.clientHeight * 0.3;
    const sheet = sheets.find(sheet => sheet.getBoundingClientRect().bottom > line) || sheets.at(-1);
    updateCurrent(Number(sheet.dataset.page));
  });
});
let resize;
window.addEventListener('resize', () => {clearTimeout(resize); resize = setTimeout(layout, 150);});
try {
  pdf = await getDocument({url:stage.dataset.pdf, isEvalSupported:false,
    standardFontDataUrl:new URL('./pdfjs/standard_fonts/', import.meta.url).href,
    wasmUrl:new URL('./pdfjs/wasm/', import.meta.url).href}).promise;
  pages = await Promise.all(Array.from({length:pdf.numPages}, (_, index) => pdf.getPage(index + 1)));
  input.max = pdf.numPages; document.querySelector('#pdf-total').textContent = `/ ${pdf.numPages}`;
  controls.forEach(control => {control.disabled = false;}); layout();
} catch {
  status.textContent = 'Unable to load the document. Download the PDF above.';
}
