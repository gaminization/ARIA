---
name: Bug Report
about: Something is broken in ARIA
labels: bug
assignees: ''
---

## Stage / Package
Stage: `stage/X` | Package: `arm_xxx`

## Describe the Bug
<!-- Clear description of what went wrong -->

## To Reproduce
```bash
# Commands to reproduce:
ros2 launch arm_bringup sim.launch.py
# then...
```

## Expected Behavior
<!-- What should have happened -->

## Actual Behavior
<!-- What actually happened -->

## Failure Classification (if applicable)
- [ ] `MISSED_OBJECT`
- [ ] `OBJECT_SLIPPED`
- [ ] `IK_FAILURE`
- [ ] `COLLISION`
- [ ] `PERCEPTION_ERROR`
- [ ] `TRACKING_LOST`
- [ ] `SERVO_FAULT`
- [ ] `COMM_TIMEOUT`
- [ ] Other: ___________

## Logs

Paste relevant output from:
ros2 topic echo /aria/state/task
or chain of thought log

## System Info
- ARIA version / git commit:
- Mode: `[ ] sim` `[ ] hardware`
- Stage completed before this: ___
