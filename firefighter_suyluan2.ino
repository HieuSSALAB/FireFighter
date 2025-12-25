#include "RandomForestModel.h"
Eloquent::ML::Port::RF rf;
boolean currentLED = 0;               // Vị trí LED đang sáng
int currentIndex = 0;
unsigned long lastUpdate = 0;
const unsigned long interval = 20; // 50Hz
#define DATA_SIZE 150
// Buffer mô phỏng dữ liệu cảm biến
float sensorBufferX[DATA_SIZE];
float sensorBufferY[DATA_SIZE];
float sensorBufferZ[DATA_SIZE];

float accX[DATA_SIZE] = {0.945, 0.945, 0.945, 0.96, 0.975, 0.975, 0.971, 0.971, 0.975, 0.975, 0.982, 0.986, 1.005, 1.005, 1.012, 1.009, 1.009, 1.012, 1.005, 1.012, 0.986, 0.971, 0.967, 0.964, 0.964, 0.975, 1.001, 1.031, 1.031, 1.065, 1.069, 1.069, 1.054, 1.054, 1.065, 1.163, 1.163, 1.238, 1.238, 1.235, 1.144, 1.144, 1.114, 1.088, 1.069, 1.02, 0.979, 0.952, 0.937, 0.915, 0.892, 0.862, 0.862, 0.858, 0.862, 0.881, 0.881, 0.903, 0.903, 0.877, 0.858, 0.858, 0.952, 0.952, 1.043, 1.043, 1.095, 1.016, 1.016, 1.005, 1.005, 1.001, 0.964, 0.964, 0.949, 0.941, 0.933, 0.922, 0.93, 0.922, 0.922, 0.922, 0.933, 0.933, 0.96, 0.964, 0.964, 0.956, 0.949, 0.949, 0.922, 0.918, 0.918, 0.941, 0.941, 0.967, 1.005, 1.005, 1.058, 1.058, 1.058, 1.009, 1.009, 0.982, 0.982, 0.997, 1.02, 1.092, 1.197, 1.302, 1.366, 1.396, 1.408, 1.359, 1.253, 1.182, 1.193, 1.265, 1.302, 1.302, 1.069, 0.866, 0.866, 0.534, 0.534, 0.55, 0.55, 0.681, 0.858, 0.858, 0.986, 0.986, 1.054, 1.065, 1.065, 0.994, 0.918, 0.885, 0.877, 0.888, 0.933, 0.975, 0.997, 1.005, 0.994, 0.982, 0.945, 0.945, 0.873, 0.858};
float accY[DATA_SIZE] = {-0.075, -0.075, -0.075, -0.06, -0.026, -0.026, -0.011, -0.011, -0.004, -0.004, 0, 0.008, 0.011, 0.011, 0.011, 0.011, 0.019, 0.011, 0.008, -0.004, -0.034, -0.064, -0.102, -0.12, -0.147, -0.162, -0.158, -0.143, -0.143, -0.068, -0.041, -0.041, 0, 0, -0.015, -0.086, -0.086, -0.102, -0.102, -0.102, -0.071, -0.071, -0.06, -0.064, -0.041, -0.034, -0.015, 0, 0.011, 0.015, -0.008, -0.019, -0.023, 0.008, 0.041, 0.083, 0.083, 0.173, 0.173, 0.248, 0.256, 0.256, 0.169, 0.169, 0.026, 0.026, -0.011, -0.071, -0.071, -0.075, -0.075, -0.068, -0.034, -0.034, -0.015, 0.03, 0.056, 0.038, 0.023, 0, 0, -0.041, -0.038, -0.019, -0.008, -0.004, 0, 0, 0, 0, -0.011, -0.011, -0.011, -0.049, -0.049, -0.075, -0.075, -0.075, 0.015, 0.015, 0.075, 0.169, 0.169, 0.184, 0.184, 0.143, 0.094, 0.064, 0.049, 0.038, 0.023, 0.038, 0.086, 0.165, 0.214, 0.252, 0.308, 0.372, 0.387, 0.387, 0.211, 0.12, 0.12, -0.045, -0.045, -0.173, -0.173, -0.192, -0.105, -0.105, 0.06, 0.06, 0.15, 0.256, 0.256, 0.308, 0.282, 0.259, 0.233, 0.184, 0.147, 0.105, 0.071, 0.023, 0.023, 0.034, 0.03, 0.03, 0.011, 0.004};
float accZ[DATA_SIZE] = {-0.021, -0.021, -0.021, -0.021, -0.052, -0.052, -0.059, -0.059, -0.056, -0.052, -0.063, -0.07, -0.087, -0.087, -0.084, -0.08, -0.073, -0.077, -0.08, -0.08, -0.091, -0.084, -0.066, -0.038, -0.038, -0.031, -0.007, -0.014, -0.014, -0.07, -0.077, -0.077, -0.052, -0.052, -0.017, 0.091, 0.091, 0.094, 0.094, 0.087, 0.08, 0.08, 0.042, 0.052, 0.049, 0.059, 0.052, 0.056, 0.017, 0.003, 0.017, 0.07, 0.115, 0.147, 0.192, 0.217, 0.217, 0.217, 0.217, 0.15, 0.07, 0.07, -0.007, -0.007, 0.014, 0.014, 0.059, 0.108, 0.108, -0.007, -0.007, -0.028, 0.014, 0.014, 0.021, 0.007, -0.003, -0.01, -0.028, -0.028, -0.028, -0.045, -0.052, -0.045, -0.066, -0.059, -0.077, -0.087, -0.087, -0.087, -0.122, -0.129, -0.129, -0.15, -0.15, -0.171, -0.168, -0.168, -0.196, -0.196, -0.196, -0.192, -0.192, -0.143, -0.143, -0.105, -0.112, -0.175, -0.272, -0.276, -0.213, -0.045, 0.049, -0.003, -0.056, -0.028, 0.042, 0.091, 0.147, 0.147, 0.227, 0.178, 0.178, 0.066, 0.066, 0.147, 0.147, 0.22, 0.255, 0.255, 0.189, 0.189, 0.15, 0.077, 0.077, 0.042, 0.038, 0.031, 0.063, 0.063, 0.059, 0.059, 0.052, 0.028, 0.01, 0.003, -0.007, -0.007, -0.059, -0.063};

#define N 150 // Số mẫu (có thể điều chỉnh tuỳ trường hợp)
//rf.predict($feat_{j}$);
struct Features {
    float max_x, ssi_x, range_z, mad_x, ene_x, mad_z, mean_x, aac_y, std_x, sma_x;
    float std_y, mead_z, mead_x, ene_y, std_z, mean_y, mad_y, iqr_y, iqr_x;
    float ssi_z, mean_z, range_x, max_z, hc_z, aac_z, sma_z, sma_y, aac_x, range_y, ssi_y;
};
#if defined(ESP32)
    #define GET_FREE_RAM() ESP.getFreeHeap()
#elif defined(ARDUINO_ARCH_AMBED)
    extern "C" uint32_t sys_get_free_heap_size(void);
    #define GET_FREE_RAM() sys_get_free_heap_size()
#else
    #define GET_FREE_RAM() 0
#endif



void setup() {
  // set the digital pin as output:
  pinMode(LED_R, OUTPUT);
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Begin test");
}
unsigned long  t_load_start =0;
void loop() {
digitalWrite(LED_R, 0);

   unsigned long now = millis();
     if (currentIndex == 0) {
      t_load_start = micros(); // BẮT ĐẦU TỪ MẪU ĐẦU TIÊN
      int ramBeforeLoad = GET_FREE_RAM();
  }
  if (now - lastUpdate >= interval) {
    lastUpdate = now;
    // --- Thu thập tín hiệu ---
    
    // Load dữ liệu mô phỏng -> buffer
    sensorBufferX[currentIndex] = accX[currentIndex];
    sensorBufferY[currentIndex] = accY[currentIndex];
    sensorBufferZ[currentIndex] = accZ[currentIndex];

    currentIndex++;

    if (currentIndex >= N) {
      unsigned long t1 = micros();
      int ramAfterLoad = GET_FREE_RAM();
      Serial.print("Data collection completed. Time (us): "); Serial.println(t1 - t_load_start);

      digitalWrite(LED_R, 1);

      unsigned long t2 = micros();
      int ramBeforeFeature = GET_FREE_RAM();

      Features result = featuresFromBuffer(sensorBufferX, sensorBufferY, sensorBufferZ, N);

      float featureArray[30] = {
        result.max_x, result.ssi_x, result.range_z, result.mad_x, result.ene_x, result.mad_z, result.mean_x, result.aac_y,
        result.std_x, result.sma_x, result.std_y, result.mead_z, result.mead_x, result.ene_y, result.std_z, result.mean_y,
        result.mad_y, result.iqr_y, result.iqr_x, result.ssi_z, result.mean_z, result.range_x, result.max_z, result.hc_z,
        result.aac_z, result.sma_z, result.sma_y, result.aac_x, result.range_y, result.ssi_y
      };

      unsigned long t3 = micros();
      int ramAfterFeature = GET_FREE_RAM();
      Serial.print("Feature extraction time (us): "); Serial.println(t3 - t2);

      unsigned long t4 = micros();
      int ramBeforePredict = GET_FREE_RAM();

      String label = rf.predictLabel(featureArray);

      unsigned long t5 = micros();
      int ramAfterPredict = GET_FREE_RAM();

      Serial.print("Prediction time (us): ");
      Serial.println(t5 - t4);

      Serial.print("Predicted label: ");
      Serial.println(label);

      while (1);
}

  }

}

unsigned long op_counter = 0;

float calcMean(float* data, int size) {
    float sum = 0;
    for(int i = 0; i < size; i++) {
        sum += data[i];
        op_counter++;  // + phép cộng
    }
    op_counter++; // phép chia
    return sum / size;
}

float calcStd(float* data, int size, float mean) {
    float sum = 0;
    for(int i = 0; i < size; i++) {
        sum += (data[i] - mean) * (data[i] - mean);
        op_counter += 3;  // 1 phép trừ + 1 phép nhân + 1 phép cộng
    }
    op_counter++; // chia
    op_counter += 3; // sqrt
    return sqrt(sum / size);
}

float calcMAD(float* data, int size, float mean) {
    float sum = 0;
    for(int i = 0; i < size; i++) {
        sum += fabs(data[i] - mean);
        op_counter += 2;  // trừ + fabs + cộng
    }
    op_counter++; // chia
    return sum / size;
}

float calcMeaD(float* data, int size) {
    float sorted[N];
    memcpy(sorted, data, size * sizeof(float));

    for(int i = 0; i < size-1; i++) {
        for(int j = i+1; j < size; j++) {
            op_counter++;  // so sánh
            if(sorted[i] > sorted[j]) {
                float temp = sorted[i];
                sorted[i] = sorted[j];
                sorted[j] = temp;
                op_counter += 3;  // hoán đổi
            }
        }
    }
    
    float median = size % 2 == 0 ? (sorted[size/2 - 1] + sorted[size/2]) / 2 : sorted[size/2];
    op_counter += 2;  // cộng + chia hoặc truy cập

    float sum = 0;
    for(int i = 0; i < size; i++) {
        sum += fabs(data[i] - median);
        op_counter += 2;  // trừ + fabs + cộng
    }
    op_counter++; // chia
    return sum / size;
}

float calcIQR(float* data, int size) {
    float sorted[N];
    memcpy(sorted, data, size * sizeof(float));

    for(int i = 0; i < size-1; i++) {
        for(int j = i+1; j < size; j++) {
            op_counter++; // so sánh
            if(sorted[i] > sorted[j]) {
                float temp = sorted[i];
                sorted[i] = sorted[j];
                sorted[j] = temp;
                op_counter += 3; // hoán đổi
            }
        }
    }

    op_counter += 2; // trừ
    return sorted[3 * size / 4] - sorted[size / 4];
}

float calcAAC(float* data, int size) {
    float sum = 0;
    for(int i = 1; i < size; i++) {
        sum += fabs(data[i] - data[i - 1]);
        op_counter += 2;  // trừ + fabs + cộng
    }
    op_counter++; // chia
    return sum / (size - 1);
}

float calcRange(float* data, int size) {
    float minV = data[0], maxV = data[0];
    for(int i = 1; i < size; i++) {
        op_counter += 2;  // so sánh
        if(data[i] > maxV) maxV = data[i];
        if(data[i] < minV) minV = data[i];
    }
    op_counter++; // trừ
    return maxV - minV;
}

float calcSSI(float* data, int size) {
    float sum = 0;
    for(int i = 0; i < size; i++) {
        sum += data[i] * data[i];
        op_counter += 2; // nhân + cộng
    }
    return sum;
}

float calcEnergy(float* data, int size) {
    op_counter += 3; // chia + sqrt
    return sqrt(calcSSI(data, size) / size);
}

float calcHjorthComplexity(float* data, int size) {
    float diff1[N-1], diff2[N-2];
    for(int i = 1; i < size; i++) {
        diff1[i-1] = data[i] - data[i-1];
        op_counter++; // trừ
    }
    for(int i = 1; i < size-1; i++) {
        diff2[i-1] = diff1[i] - diff1[i-1];
        op_counter++; // trừ
    }

    float var0 = calcStd(data, size, calcMean(data, size));
    op_counter++; // nhân
    var0 = var0 * var0;

    float var1 = calcStd(diff1, size - 1, calcMean(diff1, size - 1));
    op_counter++; // nhân
    var1 = var1 * var1;

    float var2 = calcStd(diff2, size - 2, calcMean(diff2, size - 2));
    op_counter++; // nhân
    var2 = var2 * var2;

    op_counter += 5; // chia + sqrt + chia + sqrt + chia
    return sqrt(var2 / var1) / sqrt(var1 / var0);
}

Features featuresFromBuffer(float* ax, float* ay, float* az, int size) {
    Features f;
    f.max_x = ax[0];
    f.max_z = az[0];
    f.sma_x = 0; f.sma_y = 0; f.sma_z = 0;

    for(int i = 0; i < size; i++) {
        op_counter += 2; // so sánh max
        if(ax[i] > f.max_x) f.max_x = ax[i];
        if(az[i] > f.max_z) f.max_z = az[i];
        f.sma_x += fabs(ax[i]);
        f.sma_y += fabs(ay[i]);
        f.sma_z += fabs(az[i]);
        op_counter += 3; // fabs + cộng (3 lần)
    }

    f.mean_x = calcMean(ax, size);
    f.mean_y = calcMean(ay, size);
    f.mean_z = calcMean(az, size);
    f.std_x = calcStd(ax, size, f.mean_x);
    f.std_y = calcStd(ay, size, f.mean_y);
    f.std_z = calcStd(az, size, f.mean_z);
    f.mad_x = calcMAD(ax, size, f.mean_x);
    f.mad_y = calcMAD(ay, size, f.mean_y);
    f.mad_z = calcMAD(az, size, f.mean_z);
    f.ssi_x = calcSSI(ax, size);
    f.ssi_y = calcSSI(ay, size);
    f.ssi_z = calcSSI(az, size);
    f.ene_x = calcEnergy(ax, size);
    f.ene_y = calcEnergy(ay, size);
    f.iqr_x = calcIQR(ax, size);
    f.iqr_y = calcIQR(ay, size);
    f.mead_x = calcMeaD(ax, size);
    f.mead_z = calcMeaD(az, size);
    f.range_x = calcRange(ax, size);
    f.range_y = calcRange(ay, size);
    f.range_z = calcRange(az, size);
    f.aac_x = calcAAC(ax, size);
    f.aac_y = calcAAC(ay, size);
    f.aac_z = calcAAC(az, size);
    f.hc_z = calcHjorthComplexity(az, size);

    Serial.print("Total number of operations (feature extraction)");
    Serial.println(op_counter);

    return f;
}
