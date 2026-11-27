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
measurements came from other platforms. The pretrained ViNT checkpoint
subsequently ran on an Apple M2 with MPS and CPU. With 11 windows from
the upstream GoStanford demonstration, each prediction produced five
finite waypoints and a distance value. The p95 on 200 warmed, tensor-input
forwards was 186.7 ms on MPS and 1330.6 ms on four-thread CPU. This measures
model forward only. JANUS task gain and full camera-to-command latency
still require matched episodes. One separate CPU interface check used
six rendered Gazebo JANUS frames with an independently identified goal
frame from later in the same approach. It returned five finite waypoints
and a distance value. The model was not connected to the command path.

The experiment uses 48 held-out episodes across 12 layouts, with the
models scored on matched inputs. The decision rule requires an increase
of at least 10 percentage points over the classical baseline on the
predefined junction outcome and a positive grouped confidence bound.
The proposed safety measures cover wrong-station approach, stale or
missing observations and intervention by the deterministic guard.

[Blind research and sources](https://github.com/t0ttora/me492-caster-digital-twin/blob/main/janus/vla/01-kor-arastirma.md) ·
[Independent release and inference audit](https://github.com/t0ttora/me492-caster-digital-twin/blob/main/janus/vla/02-bagimsiz-dogrulama.md) ·
[Data contract](https://github.com/t0ttora/me492-caster-digital-twin/blob/main/janus/dataset-contract.md)
