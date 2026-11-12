# ربات خودران امنیتی انبار (Autonomous Warehouse Security AMR)
### توسعه‌یافته با ROS 2 Jazzy و شبیه‌ساز Gazebo Harmonic

[![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy_Jalisco-blue.svg)](https://docs.ros.org/en/jazzy/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange.svg)](https://gazebosim.org/docs/harmonic)
[![Nav2](https://img.shields.io/badge/Navigation-Nav2-brightgreen.svg)](https://navigation.ros.org/)
[![SLAM](https://img.shields.io/badge/SLAM-SLAM__Toolbox-red.svg)](https://github.com/SteveMacenski/slam_toolbox)
[![FastAPI](https://img.shields.io/badge/Backend-FastAPI_WebSocket-009688.svg)](https://fastapi.tiangolo.com/)

[🇬🇧 English Documentation (مستندات انگلیسی)](./README.md)

---

## 📖 معرفی پروژه

این پروژه یک **ربات متحرک خودران (AMR)** کامل و صنعتی برای **نگهبانی و گشت‌زنی امنیتی هوشمند در انبارها و سوله‎های صنعتی** است. این سیستم به صورت کاملاً بومی بر روی **ROS 2 Jazzy** و شبیه‌ساز مدرن **Gazebo Harmonic (gz-sim 8)** پیاده‌سازی شده و از پشته ناوبری خودکار **Nav2**، نقشه‌برداری **SLAM Toolbox**، بینایی ماشین با **OpenCV** و یک داشبورد وب زنده با **FastAPI & WebSocket** بهره می‌برد.

### 🌟 قابلیت‌های کلیدی سیستم:
1. **اکسپلور و نقشه‌برداری کاملاً خودران (Autonomous Frontier Exploration):**
   - چرخش اولیه ۳۶۰ درجه برای مقداردهی اولیه سنسور لایدار و اسکن محیط.
   - کشف مرزهای ناشناخته نقشه به صورت هوشمند و بدون دخالت دست.
   - ذخیره خودکار نقشه انبار (`warehouse_map.yaml` و `.pgm`) و بازگشت به ایستگاه شارژ/مبدا `(0, 0, 0)`.

2. **گشت‌زنی امنیتی و گارد هوشمند (Security Patrol FSM):**
   - ماشین حالت متناهی (FSM) پیشرفته برای مدیریت گشت دوره‌ای در ایستگاه‌های مختلف انبار.
   - دید کامل ۳۶۰ درجه با دوربین‌های همزمان جلو (`/camera_front`) و عقب (`/camera_rear`).
   - در صورت مشاهده فرد مشکوک پشت سر ربات، ربات فوراً با یک چرخش سریع ۱۸۰ درجه با نفوذگر مواجه می‌شود.

3. **تشخیص نفوذگر، تفکیک پرسنل مجاز و ثبت واقعه (Incident Blackbox):**
   - تفکیک افراد متفرقه (سیلندر قرمز) از نگهبانان مجاز دارای جلیقه شبرنگ و کد شناسایی بیسیم (RFID).
   - توقف گشت و قرارگیری در وضعیت هشدار (`ALERT / INTERCEPT`).
   - عکس‌برداری فوری از صحنه (Snapshot) و ثبت رکورد قانونی در دیتابیس `SQLite` شامل زمان دقیق، موقعیت جغرافیایی `(X, Y)`، زاویه و شناسه دوربین.

4. **داشبورد مانیتورینگ تحت وب (Web Command Center):**
   - سرور ناهمگام (Asynchronous) بر پایه **FastAPI**، **WebSocket** و استریم دوگانه **MJPEG** در پورت `8080`.
   - مشاهده زنده تصویر دوربین‌های جلو و عقب با فریم‌ریت بالا.
   - نمایش زنده موقعیت ربات روی نقشه و امکان هدایت نقطه‌ای با کلیک روی نقشه (Click-to-Navigate).
   - پنل کنترل دستی جهت حرکت با کلیدهای جهت‌نما یا کیبورد (WASD).
   - دریافت آلارم‌های صوتی و تصویری آنی و تایید یا رد گزارش هشدارها توسط اپراتور.

---

## 💻 آیا هر کسی پروژه را دانلود کند می‌تواند آن را اجرا کند؟

**بله، ۱۰۰٪!** تمام پکیج‌های سورس، فایل‌های مدلسازی ربات (URDF/Xacro)، مش‌ها، متریال‌ها، محیط انبار سه‌بعدی Gazebo، نودهای هوش مصنوعی و فرانت‌اند وب در این ریپازیتوری قرار دارند و نیازی به هیچ فایل خارجی دیگری نیست.

هر فردی با داشتن **اوبونتو ۲۴.۰۴ (یا ۲۲.۰۴)** و نصب پیش‌نیازهای استاندارد ROS 2 می‌تواند مخزن را کلون کرده، بیلد کند و با یک دستور اجرا نماید.

---

## 🛠️ پیش‌نیازهای سیستمی (Prerequisites)

- **سیستم‌عامل:** Ubuntu 24.04 LTS (Noble Numbat)
- **نسخه ROS 2:** ROS 2 Jazzy Jalisco (Desktop Install)
- **شبیه‌ساز:** Gazebo Harmonic (`gz-sim 8`)
- **حداقل سخت‌افزار پیشنهادی:** ۴ گیگابایت رم و پردازنده دو هسته‌ای (در حالت `headless:=true` نیازی به کارت گرافیک مجزا نیست).

---

## 🚀 راهنمای گام‌به‌گام نصب و راه‌اندازی (برای کاربران جدید)

### ۱. کلون کردن ریپازیتوری
ترمینال را باز کرده و دستورات زیر را وارد کنید:
```bash
cd ~
git clone https://github.com/mamadixm/autonomous-warehouse-security-amr.git amr_project_ws
cd ~/amr_project_ws
```

### ۲. نصب پکیج‌ها و وابستگی‌های سیستمی
پکیج‌های مورد نیاز ROS 2 و پایتون را با دستورات زیر نصب کنید:
```bash
sudo apt update
sudo apt install -y \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-simple-commander \
    ros-jazzy-slam-toolbox \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-cv-bridge \
    ros-jazzy-teleop-twist-keyboard \
    python3-pip python3-opencv

# نصب کتابخانه‌های سرور و وب با pip
pip install --break-system-packages fastapi uvicorn websockets pillow
```

### ۳. بیلد و کامپایل ورک‌اسپیس
```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

---

## 🎮 نحوه اجرای پروژه

### روش اول: اجرای تک‌دستوری کل مأموریت (پیشنهادی 🌟)
این دستور ارکستریتور تمام‌خودکار پروژه را صدا می‌زند:
1. ابتدا محیط انبار شبیه‌سازی شده را بالا می‌آورد.
2. سرور وب و دوربین‌ها در `http://localhost:8080` فعال می‌شوند.
3. ربات یک چرخش ۳۶۰ درجه انجام داده، انبار را کامل اسکن و نقشه را ذخیره می‌کند.
4. به مبدا برمی‌گردد و سپس وارد فاز گشت‌زنی امنیتی پایدار می‌شود:

```bash
# بستن پروسه‌های معلق قبلی (در صورت وجود)
pkill -9 -f "gz sim|ros2|auto_explorer|command_center"

source /opt/ros/jazzy/setup.bash
source ~/amr_project_ws/install/setup.bash

# حالت بهینه و کم‌مصرف (بدون درگیر کردن گرافیک، مناسب هر سیستمی)
ros2 launch amr_command_center full_demo.launch.py headless:=true rviz:=false
```

> **نکته:** اگر سیستم شما کارت گرافیک قوی دارد و مایلید محیط سه‌بعدی Gazebo و نمای RViz2 را با چشم ببینید، پرچم‌ها را بردارید:
> ```bash
> ros2 launch amr_command_center full_demo.launch.py
> ```

پس از اجرا، مرورگر خود را باز کنید و وارد آدرس زیر شوید:
👉 **[http://localhost:8080](http://localhost:8080)**

---

### روش دوم: اجرای مرحله‌به‌مرحله و ماژولار

#### مرحله ۱: نقشه‌برداری خودکار انبار
```bash
# ترمینال ۱: اجرای ناوبری و اکتشاف خودکار
source /opt/ros/jazzy/setup.bash
source ~/amr_project_ws/install/setup.bash
ros2 launch amr_navigation auto_mapping.launch.py
```
*ربات شروع به چرخش و حرکت در تمام فضاهای خالی کرده، نقشه را در `src/amr_navigation/maps/warehouse_map` ذخیره می‌کند و به نقطه (0,0) بازمی‌گردد.*

#### مرحله ۲: شروع گشت امنیتی و مرکز مانیتورینگ وب
```bash
# ترمینال ۱: اجرای شبیه‌ساز انبار
source /opt/ros/jazzy/setup.bash
source ~/amr_project_ws/install/setup.bash
ros2 launch amr_gazebo warehouse.launch.py

# ترمینال ۲: اجرای مرکز فرماندهی و گشت‌زنی هوشمند
source /opt/ros/jazzy/setup.bash
source ~/amr_project_ws/install/setup.bash
ros2 launch amr_command_center security_command_center.launch.py
```

---

## 🧪 سناریوهای تست و تزریق رویدادهای امنیتی

برای تست هوش ربات در واکنش به خطرات، اسکریپت‌های زیر در پوشه `src/amr_gazebo/scripts/` قرار دارند:

1. **اسپاون نفوذگر غیرمجاز در برابر دید ربات (سیلندر قرمز):**
   ```bash
   ~/amr_project_ws/src/amr_gazebo/scripts/spawn_intruder.sh 1.5 0.0 0.35
   ```
   *ربات نفوذگر را می‌بیند، آژیر هشدار در وب به صدا درمی‌آید، واقعه در SQLite ثبت و عکس صحنه ذخیره می‌شود.*

2. **تست دوربین عقب (چرخش ۱۸۰ درجه ربات):**
   نفوذگر را پشت سر ربات ظاهر کنید:
   ```bash
   ~/amr_project_ws/src/amr_gazebo/scripts/spawn_intruder.sh -1.5 0.0 0.35
   ```
   *ربات از طریق دوربین عقب نفوذگر را شناسایی کرده و در جا ۱۸۰ درجه می‌چرخد تا با سنسورهای جلویی با او مقابله کند.*

3. **اسپاون نگهبان مجاز (دارای جلیقه سبز و کد RFID معتبر):**
   ```bash
   ~/amr_project_ws/src/amr_gazebo/scripts/spawn_guard.sh 1.5 0.0 0.35
   ```
   *ربات کارت و جلیقه را اعتبارسنجی کرده، پیام خوش‌آمدگویی ثبت می‌کند و بدون ایجاد آلارم به گشت ادامه می‌دهد.*

4. **پاک‌سازی مدل‌های تست از محیط انبار:**
   ```bash
   ~/amr_project_ws/src/amr_gazebo/scripts/spawn_intruder.sh delete
   ~/amr_project_ws/src/amr_gazebo/scripts/spawn_guard.sh delete
   ```

---

## 📁 ساختار پکیج‌های ورک‌اسپیس

- **`amr_description`**: فایل‌های URDF و Xacro ساختار بدنه، چرخ‌ها، مفصل‌ها، سنسور لایدار دو بعدی و دوربین‌های واید جلو و عقب.
- **`amr_gazebo`**: جهان شبیه‌سازی انبار صنعتی (`depot.sdf`)، پل ارتباطی `ros_gz_bridge` و اسکریپت‌های تست نفوذ.
- **`amr_navigation`**: پیکربندی الگوریتم ناوبری Nav2، پکیج نقشه‌برداری آنلاین SLAM Toolbox، نود کاوش خودکار (`auto_explorer_node`) و نود هوش مصنوعی گشت (`patrol_interceptor_node`).
- **`amr_command_center`**: سرور بک‌اند FastAPI و WebSocket، استریم ویدئویی MJPEG، رابط کاربری دارک تحت وب (`web/index.html`) و ارکستریتور کل پروژه.

---

## 🤝 نحوه توسعه و مشارکت
اگر توسعه‌دهنده دیگری هستید و مایلید این پروژه را گسترش دهید:
1. پروژه را Fork یا Clone کنید.
2. یک Branch جدید برای ویژگی خود بسازید (`git checkout -b feature/AmazingFeature`).
3. تغییرات خود را Commit کرده و Push کنید (`git push origin feature/AmazingFeature`).
4. یک Pull Request ارسال فرمایید.
