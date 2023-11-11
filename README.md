# Ardupilot Rover Build

### Hardware
- Base Vehicle: [Losi DBXL-E 2.0](https://www.losi.com/product/1-5-dbxl-e-2.0-4x4-desert-buggy-brushless-rtr-with-smart-fox/LOS05020V2T1.html)

- Companion Computer: [Jetson Nano 4GB](https://developer.nvidia.com/embedded/jetson-nano-developer-kit)
    - Power Module: [18650 Single Cell Li-Ion Shield](https://a.co/d/eSGOiVn)

- Flight Controller: [Pixhawk 6C](https://www.getfpv.com/holybro-pixhawk-6c-pm07-power-module-m8n-gps.html)

- Camera: [OAK-D-Lite](https://shop.luxonis.com/products/oak-d-lite-1?variant=42583102456031)

Jetson -> Pixhawk Wiring: [YouTube Video](https://youtu.be/nIuoCYauW3s?si=iFBYiQlUnzNoR8pS)

---

OAK-D-Lite examples from: https://github.com/luxonis/depthai-python/tree/main/examples

---

To start up UDP streaming from Jetson over SSH:
```
sudo mavproxy.py --master=/dev/ttyTHS1 --out=udp:<GCS IP>:14550
```

![Rover](imgs/IMG_6536.jpg)

![Rover](imgs/IMG_6537.jpg)

