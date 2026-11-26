# Visual navigation shortlist and test plan

Two independent reviews considered pretrained navigation policies for
JANUS. The first experiment is ViNT for visual staging, with GNM as
the smaller learned comparison. The decision baseline is Nav2 to a
mapped staging pose followed by the classical QR approach. A frozen
ViNT policy would use the six-frame history, taught views and a
goal photograph. It would propose waypoints before the QR handoff.
OmniVLA-Edge is the first language-conditioned candidate if a
separate task requires language to resolve the destination.

Official ViNT and GNM source was accessible, and the publisher-linked
weight files returned public attachment metadata. Separate checkpoint
redistribution terms are unresolved. Published speed and accuracy
measurements came from other platforms. The target systems did not have
a prepared inference runtime, matching available weights and a suitable
JANUS episode for an end-to-end smoke test.

The experiment uses 48 held-out episodes across 12 layouts, with the
models scored on matched inputs. The decision rule requires an increase
of at least 10 percentage points over the classical baseline on the
predefined junction outcome and a positive grouped confidence bound.
The proposed safety measures cover wrong-station approach, stale or
missing observations and intervention by the deterministic guard.

[Blind research and sources](https://github.com/t0ttora/me492-caster-digital-twin/blob/main/janus/vla/01-kor-arastirma.md) ·
[Independent release and inference audit](https://github.com/t0ttora/me492-caster-digital-twin/blob/main/janus/vla/02-bagimsiz-dogrulama.md) ·
[Data contract](https://github.com/t0ttora/me492-caster-digital-twin/blob/main/janus/dataset-contract.md)
