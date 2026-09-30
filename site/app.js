import {issueStatus, validIssue} from './assets/status.mjs';
const tasks = [...document.querySelectorAll('.task')];
const search = document.querySelector('#task-search');
const filters = [...document.querySelectorAll('[data-filter]')];
let filter = 'all';
function updateTasks() {
  if (!search) return;
  const query = search.value.toLocaleLowerCase('tr').trim();
  let visible = 0;
  for (const task of tasks) {
    const match = (filter === 'all' || task.dataset.status === filter || (filter === 'gate' && task.dataset.kind === 'gate')) && task.textContent.toLocaleLowerCase('tr').includes(query);
    task.hidden = !match;
    if (match) visible++;
  }
  document.querySelector('#task-count').textContent = `${visible} / ${tasks.length} görev gösteriliyor`;
  document.querySelector('#empty-tasks').hidden = visible > 0;
}
if (search) {
  search.addEventListener('input', updateTasks);
  for (const button of filters) button.addEventListener('click', () => {
    filter = button.dataset.filter;
    for (const item of filters) item.setAttribute('aria-pressed', String(item === button));
    updateTasks();
  });
  updateTasks();
}
function openLinkedTask() {
  const id = location.hash.slice(1);
  if (!/^task-\d+$/.test(id)) return;
  const task = document.getElementById(id);
  if (!task) return;
  filter = 'all';
  if (search) search.value = '';
  for (const button of filters) button.setAttribute('aria-pressed', String(button.dataset.filter === 'all'));
  updateTasks();
  task.open = true;
}
if (search) {
  openLinkedTask();
  window.addEventListener('hashchange', openLinkedTask);
}
// Preserve old shared section URLs while all navigation uses real pages.
const oldPage = location.hash.slice(1);
if (/\/(?:index\.html)?$/.test(location.pathname) && ['progress','vision','manifesto','tasks','roadmap','decisions','library','about'].includes(oldPage)) location.replace(`${oldPage}.html`);

const refresh = document.querySelector('#refresh');
const syncStatus = document.querySelector('#sync-status');
const labels = {planned:'Planlanan','in-progress':'Devam ediyor',blocked:'Engelli',verified:'Doğrulandı',closed:'Kapalı · doğrulanmadı',unlabelled:'Durum belirtilmedi',conflicting:'Etiketler çelişkili'};
if (refresh) {
  refresh.hidden = false;
  refresh.addEventListener('click', async () => {
    refresh.disabled = true;
    refresh.textContent = 'GitHub kontrol ediliyor…';
    const controller = new AbortController();
    const timeout = setTimeout(() => controller.abort(), 15000);
    try {
      const issues = [];
      for (let page = 1; page <= 10; page++) {
        const response = await fetch(`https://api.github.com/repos/t0ttora/me492-caster-digital-twin/issues?state=all&per_page=100&page=${page}`, {headers:{Accept:'application/vnd.github+json'},signal:controller.signal});
        if (!response.ok) throw new Error('GitHub unavailable');
        const batch = await response.json();
        if (!Array.isArray(batch)) throw new Error('Invalid response');
        const records = batch.filter(issue => !issue.pull_request);
        if (!records.every(validIssue)) throw new Error('Invalid issue data');
        issues.push(...records);
        if (batch.length < 100) break;
        if (page === 10) throw new Error('Incomplete issue list');
      }
      const byNumber = new Map(issues.map(issue => [issue.number, issue]));
      for (const task of tasks) {
        const issue = byNumber.get(Number(task.dataset.issue));
        const state = issue ? issueStatus(issue) : 'unlabelled';
        task.dataset.status = state;
        const badge = task.querySelector('.badge');
        badge.className = `badge ${state}`;
        badge.textContent = labels[state];
      }
      const weekly = issues.filter(issue => issue.labels.some(label => (typeof label === 'string' ? label : label.name) === 'type:weekly'));
      const values = [weekly.filter(i => issueStatus(i) === 'in-progress').length, weekly.filter(i => issueStatus(i) === 'verified').length, weekly.filter(i => issueStatus(i) === 'blocked').length, weekly.length];
      document.querySelectorAll('.metric-value').forEach((item, index) => {item.textContent = String(values[index]).padStart(2, '0');});
      const focus = document.querySelector('#focus-items');
      if (focus) focus.replaceChildren();
      const active = weekly.filter(issue => issueStatus(issue) === 'in-progress');
      for (const issue of active) {
        const heading = document.createElement('h3'); heading.textContent = issue.title.split('|').at(-1).trim();
        const badge = document.createElement('span'); badge.className = 'badge in-progress'; badge.textContent = labels['in-progress'];
        const paragraph = document.createElement('p'); const link = document.createElement('a'); link.href = issue.html_url; link.textContent = `Görev #${issue.number} ↗`; paragraph.append(link);
        if (focus) focus.append(heading, badge, paragraph);
      }
      if (focus && !active.length) {const heading = document.createElement('h3');heading.textContent = 'Aktif görev işaretlenmedi.';focus.append(heading);}
      const checked = new Intl.DateTimeFormat('tr-TR',{dateStyle:'short',timeStyle:'short',timeZone:'Europe/Istanbul'}).format(new Date());
      syncStatus.textContent = `Görev durumları GitHub’dan yenilendi · ${checked} (İstanbul). Araştırma ve yorum içerikleri son yayının kaydıdır.`;
      updateTasks();
    } catch {
      syncStatus.textContent = 'GitHub yenilemesi kullanılamıyor. Son gösterilen kayıt korunuyor; güncel durum için GitHub’ı açın.';
    } finally {
      clearTimeout(timeout);
      refresh.disabled = false;
      refresh.textContent = 'GitHub durumunu yenile ↻';
    }
  });
}

const currentLink = document.querySelector('.header nav [aria-current="page"]');
const navigation = document.querySelector('.header nav');
if (currentLink && navigation && navigation.scrollWidth > navigation.clientWidth) navigation.scrollLeft = currentLink.offsetLeft - navigation.offsetLeft - navigation.clientWidth / 2 + currentLink.clientWidth / 2;
