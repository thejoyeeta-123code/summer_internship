summer internship project codebase

create an empty directory components and git clone myles parfeniuk's bno08x library for esp32p4 inside it, then build the project

link to required git repo - https://github.com/myles-parfeniuk/esp32_BNO08x


OVERVIEW:
The Smart Interactive Doll is a multimodal, sensor-integrated toy prototype designed to recognize behavioral patterns from motion and audio signals.

The system combines a BNO085 inertial measurement unit (IMU) with audio acquisition capabilities on the ESP32-P4 platform. The collected signals are processed to extract meaningful features, which are then used by machine learning models to classify different activities.
The broader research motivation is to explore privacy-conscious, low-cost interactive toys that can observe patterns of play and potentially support early developmental monitoring research.

PRIMARY OBJECTIVE OF THE PROJECT :
 develop a sensor-integrated smart toy prototype 
 collect motion and audio data during different interactions.
 develop a dataset containing multiple activity classes.
 perform feature engineering and dimensionality reduction.
 train and evaluate machine learning classifiers.
 explore the deployment of trained models on embedded hardware.
 communicate relevant system information through BLE.
 develop a dashboard for monitoring data and inference results.


SYSTEM FLOW 
        SMART INTERACTIVE DOLL
                     |
          +----------+----------+
          |                     |
     Motion Sensor         Audio Input
       BNO085              ESP32-P4
          |                     |
          |              Audio Acquisition
          |                     |
     IMU Data               Audio Data
          |                     |
          +----------+----------+
                     |
              Signal Processing
                     |
             Feature Extraction
                     |
         Feature Selection / PCA
                     |
           Machine Learning Model
                     |
             Activity Prediction
                     |
          +----------+----------+
          |                     |
    Embedded Output       BLE Communication
                                |
                         Web Dashboard
                                |
                    Prediction Visualization

Hardware Components
he ESP32-P4 serves as the main embedded processing platform.
The BNO085 is a 9-axis inertial measurement unit with onboard sensor-fusion capabilities.
Software and Technologies
C/C++ ----Embedded firmware development

ESP-IDF----ESP32 firmware development framework

Python---Dataset processing and model development

NumPy and pandas---Numerical operations and dataset manipulation

scikit-learn----Machine learning and model evaluation

LOSO----STATISTICAL METHOD 

BLE---Wireless communication

HTML, CSS, and JavaScript---Dashboard technologies, if used by the current implementation




 

