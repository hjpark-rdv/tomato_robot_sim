# Third-party notices

## LaboroTomato

The optional stage-2 cherry-tomato instance segmentation feature uses the
[laboroai/LaboroTomato](https://github.com/laboroai/LaboroTomato) model
architecture example and the published `laboro_tomato_little_48ep.pth`
checkpoint.

- Copyright: Laboro.AI Inc. and LaboroTomato contributors
- License: Creative Commons Attribution-NonCommercial-ShareAlike 4.0
  International (CC BY-NC-SA 4.0)
- Upstream revision used while integrating: `1afeb891086dedde0f2ee153d4df125a21852b13`
- Model SHA-256:
  `694ca8a9606124ffe36f5316fb34170299c25333f64c2a1ece9e5cc8ac4b7ca6`

The checkpoint, PyTorch virtual environment, and upstream repository clone are
not stored in this Git repository. Run `scripts/setup_laboro_tomato.sh` to
install/download them locally.

The upstream license restricts this feature to non-commercial use. Commercial
deployment requires separate permission from Laboro.AI Inc.
