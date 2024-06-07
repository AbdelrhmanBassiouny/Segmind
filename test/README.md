# EpisodeSegmenter

A python library for segmenting simulation episodes of activities. This is done by detecting physical interactions,
and events in the simulation.
This library also integrates with [NEEMPycramInterface](https://github.com/AbdelrhmanBassiouny/NEEMPyCRAMInterface) to segment NEEM episodes as a use case.
## Installation

```bash
pip install episode_segmenter
```

## Example Usage

All below examples assume the neems are located in a 'test' database at 'localhost' which can be accessed by 'newuser'
using password 'password'.

### Replaying the motions of a NEEM and segmenting it:

This is done by using the PyCRAMNEEMInterface class which provides an easy way to replay the motions of a NEEM,
then using the NEEMSegmenter class which implements EpisodeSegmenter to segment the motions into activities, actions,
and events.

```Python
from neem_pycram_interface import PyCRAMNEEMInterface
from episode_segmenter.neem_segmenter import NEEMSegmenter

from pycram.datastructures.enums import WorldMode
from pycram.worlds.bullet_world import BulletWorld

BulletWorld(WorldMode.GUI)
pni = PyCRAMNEEMInterface('mysql+pymysql://newuser:password@localhost/test')
ns = NEEMSegmenter(pni, annotate_events=True)
ns.run_event_detectors_on_neem([15])
```