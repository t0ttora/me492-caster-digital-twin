import { test } from 'node:test';
import assert from 'node:assert/strict';
import { issueStatus, matchesFilter, validIssue, formatDate } from '../site/assets/status.mjs';

test('explicit status labels remain distinct from closure', () => {
  assert.equal(issueStatus({state:'open', labels:['status:in-progress']}), 'in-progress');
  assert.equal(issueStatus({state:'closed', labels:[]}), 'closed');
  assert.equal(issueStatus({state:'closed', labels:[{name:'status:verified'}]}), 'verified');
  assert.equal(issueStatus({state:'open', labels:['status:blocked']}), 'blocked');
  assert.equal(issueStatus({state:'open', labels:[]}), 'unlabelled');
  assert.equal(issueStatus({state:'open', labels:['status:blocked','status:verified']}), 'conflicting');
});
test('search and status combine without losing the source row', () => {
  assert.equal(matchesFilter('Caster model and pilot','planned','all','CASTER'), true);
  assert.equal(matchesFilter('Caster model and pilot','planned','verified','caster'), false);
  assert.equal(matchesFilter('Caster model and pilot','planned','planned','  '), true);
});
test('only well-formed expected repository issues enter the live view', () => {
  assert.equal(validIssue({number:1,state:'open',labels:[],html_url:'https://github.com/t0ttora/me492-caster-digital-twin/issues/1'}), true);
  assert.equal(validIssue({number:1,state:'open',labels:[],html_url:'javascript:alert(1)'}), false);
  assert.equal(validIssue({number:1,state:'open',labels:[],html_url:'https://github.com/other/repo/issues/1'}), false);
});
test('bad date is explicit rather than fabricated', () => {
  assert.equal(formatDate('wrong'), 'Date unavailable');
  assert.match(formatDate('2026-09-30T00:00:00Z'), /30 Sept? 2026/);
});
