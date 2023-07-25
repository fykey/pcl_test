# インストール手順

## librealsense
```bash
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt-get update

sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
sudo apt-get install librealsense2-dev
sudo apt-get install librealsense2-dbg
```

* 参考サイト
    * https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md

## OpenCV

```bash
sudo apt install libopencv-dev
```

## PCL

```bash
sudo apt install libpcl-dev
```

# Viewer

## CloudCompare

* install

```bash
sudo apt update
sudo apt install snapd
sudo snap install cloudcompare
```

* launch

```bash
cloudcomare.CloudCompare
```
