# ballcollector
ece4191 project overview:

synopsis:
- be able to locate tennis balls contained within its quadrant of the court
- be able to move to a located tennis ball
- be able to pick up located tennis balls
- be able to navigate to a collection box located in the center of the four quadrants of the court
- be able to deposit collected tennis balls in the cardboard collection box of dimensions 60x45x18cm
- may travel into other quadrants of the court momentarily

## Pi Setup

Environment initialisation:

```sh
sudo apt-get update && sudo apt-get upgrade -y
sudo apt-get install -y git libgl1
```

To install Python packages:

```sh
python3.11 -m venv .venv
. .venv/bin/activate
pip install -r requirements.txt
```

To get proper headless opencv

```sh
pip uninstall opencv-python && pip install opencv-python-headless==4.10.0.84
```