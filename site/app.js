import {issueStatus, validIssue} from './assets/status.mjs';
const tasks = [...document.querySelectorAll('.task')];
const search = document.querySelector('#task-search');
const filters = [...document.querySelectorAll('[data-filter]')];
let filter = 'all', kind = 'all', selectedTask;
const kindFilters = [...document.querySelectorAll('[data-kind-filter]')];
const inspector = document.querySelector('#task-inspector');
function updateTasks() {
  if (!search) return;
  const query = search.value.toLocaleLowerCase('en').trim();
  const matchesSearch = task => task.textContent.toLocaleLowerCase('en').includes(query);
  let visible = 0;
  for (const task of tasks) {
    const match = (filter === 'all' || task.dataset.status === filter) && (kind === 'all' || task.dataset.kind === kind) && matchesSearch(task);
    task.hidden = !match;
    if (!match) task.open = false;
    if (match) visible++;
  }
  for (const button of filters) {
    button.setAttribute('aria-pressed', String(button.dataset.filter === filter));
    button.querySelector('.filter-count').textContent = tasks.filter(task => (kind === 'all' || task.dataset.kind === kind) && matchesSearch(task) && (button.dataset.filter === 'all' || task.dataset.status === button.dataset.filter)).length;
  }
  for (const button of kindFilters) {
    button.setAttribute('aria-pressed', String(button.dataset.kindFilter === kind));
    button.querySelector('.filter-count').textContent = tasks.filter(task => task.dataset.kind === button.dataset.kindFilter || button.dataset.kindFilter === 'all').length;
  }
  const statusName = filter === 'all' ? 'All statuses' : document.querySelector(`[data-filter="${filter}"]`).getAttribute('aria-label');
  document.querySelector('#task-count').textContent = `${visible} of ${tasks.length} records · ${statusName}${query ? ` · “${search.value.trim()}”` : ''}`;
  document.querySelector('#reset-tasks').hidden = filter === 'all' && kind === 'all' && !query;
  document.querySelector('#empty-tasks').hidden = visible > 0;
}
function showTask(task) {
  selectedTask = task;
  tasks.forEach(item => item.classList.toggle('is-selected', item === task));
  document.querySelector('#inspector-title').textContent = task.querySelector('.task-title').textContent;
  document.querySelector('#inspector-code').textContent = `${task.querySelector('.task-code').textContent} / ${task.querySelector('.task-meta').textContent}`;
  document.querySelector('#inspector-status').replaceChildren(task.querySelector('.badge').cloneNode(true));
  const body = task.querySelector('.task-body').cloneNode(true);
  body.querySelector('[data-close-task]').remove();
  body.querySelector('.task-actions a').href = `${location.origin}${location.pathname}#${task.id}`;
  document.querySelector('#inspector-content').replaceChildren(body);
  const visible = tasks.filter(item => !item.hidden), index = visible.indexOf(task);
  document.querySelector('#inspector-position').textContent = `${index + 1} of ${visible.length}`;
  document.querySelector('#task-prev').disabled = index <= 0;
  document.querySelector('#task-next').disabled = index >= visible.length - 1;
  history.replaceState(null, '', `${location.pathname}${location.search}#${task.id}`);
  if (!inspector.open) {document.body.classList.add('task-detail-open'); inspector.showModal();}
  document.querySelector('#inspector-content').scrollTop = 0;
}
function openLinkedTask() {
  const id = location.hash.slice(1);
  const issueList = document.getElementById('semester-issues');
  if (id === 'semester-issues') issueList.open = true;
  if (!/^task-\d+$/.test(id)) return;
  const task = document.getElementById(id);
  if (!task) return;
  issueList.open = true;
  filter = 'all'; kind = 'all'; search.value = ''; updateTasks(); showTask(task);
}
if (search) {
  search.addEventListener('input', updateTasks);
  filters.forEach(button => button.addEventListener('click', () => {filter = button.dataset.filter; updateTasks();}));
  kindFilters.forEach(button => button.addEventListener('click', () => {kind = button.dataset.kindFilter; updateTasks();}));
  document.querySelector('#reset-tasks').addEventListener('click', () => {filter = 'all'; kind = 'all'; search.value = ''; updateTasks(); search.focus();});
  for (const task of tasks) {
    const summary = task.querySelector('summary');
    summary.setAttribute('aria-haspopup', 'dialog');
    summary.addEventListener('click', event => {event.preventDefault(); showTask(task);});
  }
  inspector.addEventListener('close', () => {
    document.body.classList.remove('task-detail-open');
    tasks.forEach(task => task.classList.remove('is-selected'));
    history.replaceState(null, '', location.pathname + location.search);
    selectedTask?.querySelector('summary').focus({preventScroll:true});
  });
  inspector.addEventListener('click', event => {if (event.target === inspector) {const rect = inspector.getBoundingClientRect(); if (event.clientX < rect.left || event.clientX > rect.right || event.clientY < rect.top || event.clientY > rect.bottom) inspector.close();}});
  for (const [id, step] of [['task-prev', -1], ['task-next', 1]]) document.querySelector(`#${id}`).addEventListener('click', () => {
    const visible = tasks.filter(task => !task.hidden), index = visible.indexOf(selectedTask);
    if (visible[index + step]) showTask(visible[index + step]);
  });
  updateTasks(); openLinkedTask();
  window.addEventListener('hashchange', openLinkedTask);
}
// Preserve old shared section URLs while all navigation uses real pages.
const oldPage = location.hash.slice(1);
if (/\/(?:index\.html)?$/.test(location.pathname) && ['progress','vision','manifesto','tasks','roadmap','decisions','library','about'].includes(oldPage)) location.replace(`${oldPage === 'manifesto' ? 'vision' : oldPage}.html`);

if (document.body.classList.contains('page-results')) {
  for (const entry of document.querySelectorAll('.case-entry')) {
    const video = entry.querySelector('[data-case-video]');
    entry.addEventListener('toggle', () => {
      if (entry.open && !video.hasAttribute('src')) {
        if (video.dataset.poster) video.poster = video.dataset.poster;
        video.src = video.dataset.caseVideo;
      }
      if (!entry.open) video.pause();
    });
  }
  const openLinkedCase = () => {
    const target = document.getElementById(location.hash.slice(1));
    if (target?.classList.contains('case-entry')) {
      target.open = true;
      requestAnimationFrame(() => target.scrollIntoView({block:'start'}));
    }
  };
  openLinkedCase();
  window.addEventListener('hashchange', openLinkedCase);
}

const refresh = document.querySelector('#refresh');
const syncStatus = document.querySelector('#sync-status');
const labels = {planned:'Planned','in-progress':'In progress',blocked:'Blocked',verified:'Verified',closed:'Closed · unverified',unlabelled:'Unclassified',conflicting:'Conflicting labels'};
if (refresh) {
  refresh.hidden = false;
  refresh.addEventListener('click', async () => {
    refresh.disabled = true;
    refresh.textContent = 'Checking GitHub…';
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
      for (const record of document.querySelectorAll('.task, [data-progress-issue]')) {
        const issue = byNumber.get(Number(record.dataset.issue ?? record.dataset.progressIssue));
        const state = issue ? issueStatus(issue) : 'unlabelled';
        record.dataset.status = state;
        const badge = record.querySelector('.badge');
        badge.className = `badge ${state}`;
        badge.textContent = labels[state];
      }
      const weekly = issues.filter(issue => issue.labels.some(label => (typeof label === 'string' ? label : label.name) === 'type:weekly'));
      const focus = document.querySelector('#focus-items');
      if (focus) focus.replaceChildren();
      const active = weekly.filter(issue => issueStatus(issue) === 'in-progress');
      for (const issue of active) {
        const heading = document.createElement('h3'); heading.textContent = issue.title.split('|').at(-1).trim();
        const badge = document.createElement('span'); badge.className = 'badge in-progress'; badge.textContent = labels['in-progress'];
        const paragraph = document.createElement('p'); const link = document.createElement('a'); link.href = `tasks.html#task-${issue.number}`; link.textContent = `Task #${issue.number} ↗`; paragraph.append(link);
        if (focus) focus.append(heading, badge, paragraph);
      }
      if (focus && !active.length) {const heading = document.createElement('h3');heading.textContent = 'No task is marked in progress.';focus.append(heading);}
      const checked = new Intl.DateTimeFormat('en-GB',{dateStyle:'short',timeStyle:'short',timeZone:'Europe/Istanbul'}).format(new Date());
      syncStatus.textContent = `Task statuses refreshed from GitHub · ${checked} (Istanbul). Evidence notes and comments reflect the last published snapshot.`;
      updateTasks();
      if (inspector?.open && selectedTask) showTask(selectedTask);
    } catch {
      syncStatus.textContent = 'GitHub refresh is unavailable. The displayed snapshot is retained. Open GitHub for the current status.';
    } finally {
      clearTimeout(timeout);
      refresh.disabled = false;
      refresh.textContent = 'Refresh GitHub status';
    }
  });
}

const currentLink = document.querySelector('.header nav [aria-current="page"]');
const navigation = document.querySelector('.header nav');
if (currentLink && navigation && navigation.scrollWidth > navigation.clientWidth) navigation.scrollLeft = currentLink.offsetLeft - navigation.offsetLeft - navigation.clientWidth / 2 + currentLink.clientWidth / 2;

const menuToggle = document.querySelector('.nav-toggle');
if (menuToggle) {
  menuToggle.hidden = false;
  menuToggle.addEventListener('click', () => {
    const expanded = menuToggle.getAttribute('aria-expanded') !== 'true';
    menuToggle.setAttribute('aria-expanded', String(expanded));
    menuToggle.textContent = expanded ? 'Close ×' : 'Menu +';
  });
  navigation.addEventListener('keydown', event => {
    if (event.key === 'Escape' && menuToggle.getAttribute('aria-expanded') === 'true') {
      menuToggle.click(); menuToggle.focus();
    }
  });
}

const weekDetails = document.querySelector('details#weekly-progress');
if (weekDetails) {
  function showWeek() {
    if (location.hash === '#weekly-progress' || /^#week-W\d+$/.test(location.hash)) {
      weekDetails.open = true;
      requestAnimationFrame(() => document.getElementById(location.hash.slice(1))?.scrollIntoView());
    }
  }
  showWeek();
  window.addEventListener('hashchange', showWeek);
}

const librarySearch = document.querySelector('#library-search');
if (librarySearch) {
  librarySearch.closest('label').hidden = false;
  const rows = [...document.querySelectorAll('#research-index tbody tr')];
  librarySearch.addEventListener('input', () => {
    const query = librarySearch.value.trim().toLowerCase();
    for (const row of rows) row.hidden = !row.textContent.toLowerCase().includes(query);
    const count = rows.filter(row => !row.hidden).length;
    document.querySelector('#library-count').textContent = query ? `${count} matching records` : '';
  });
}

const figureViewer = document.querySelector('#figure-viewer');
if (figureViewer?.showModal) {
  const stage = document.querySelector('#figure-stage');
  const image = document.querySelector('#figure-image');
  const title = document.querySelector('#figure-title');
  const caption = document.querySelector('#figure-caption');
  const message = document.querySelector('#figure-message');
  const original = document.querySelector('#figure-original');
  const scale = document.querySelector('#figure-scale');
  let opener;

  function resetScale() {
    stage.classList.remove('figure-actual');
    stage.scrollTo(0, 0);
    scale.textContent = 'Actual size';
  }

  image.addEventListener('load', () => {
    image.hidden = false;
    message.hidden = true;
    stage.setAttribute('aria-busy', 'false');
    scale.disabled = false;
  });
  image.addEventListener('error', () => {
    message.textContent = 'The figure could not load. Open the original to try again.';
    stage.setAttribute('aria-busy', 'false');
  });
  scale.addEventListener('click', () => {
    if (stage.classList.contains('figure-actual')) resetScale();
    else {
      stage.classList.add('figure-actual');
      scale.textContent = 'Fit to screen';
    }
  });

  for (const link of document.querySelectorAll('main a[href]:not([download])')) {
    const url = new URL(link.href);
    if (url.origin !== location.origin || !/\.(png|jpe?g|svg|webp)$/i.test(url.pathname)) continue;
    link.setAttribute('aria-haspopup', 'dialog');
    link.addEventListener('click', event => {
      if (event.defaultPrevented || event.button !== 0 || event.metaKey || event.ctrlKey || event.shiftKey || event.altKey) return;
      event.preventDefault();
      opener = link;
      const previewAlt = link.querySelector('img')?.alt;
      const description = link.closest('figure')?.querySelector('figcaption');
      const heading = description?.querySelector('strong')?.textContent.trim();
      title.textContent = heading || previewAlt || link.textContent.trim() || 'Research figure';
      const text = description?.textContent.trim() || '';
      caption.textContent = heading && text.startsWith(heading) ? text.slice(heading.length).trim() : text;
      caption.hidden = !caption.textContent;
      original.href = url.href;
      image.alt = previewAlt || '';
      image.hidden = true;
      message.textContent = 'Loading figure…';
      message.hidden = false;
      stage.setAttribute('aria-busy', 'true');
      scale.disabled = true;
      resetScale();
      figureViewer.showModal();
      image.src = url.href;
    });
  }

  figureViewer.addEventListener('click', event => {
    if (event.target !== figureViewer) return;
    const {left, right, top, bottom} = figureViewer.getBoundingClientRect();
    if (event.clientX < left || event.clientX > right || event.clientY < top || event.clientY > bottom) figureViewer.close();
  });
  figureViewer.addEventListener('close', () => opener?.focus({preventScroll: true}));
}
