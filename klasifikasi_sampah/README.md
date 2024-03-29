# Repo Klasifikasi Sampah

## Cara Pakai

1. clone repository
    ```
    git clone https://github.com/ganesisx/Dokumentasi-KRI.git
    ```
2. Masuk ke folder klasifikasi_sampah
    ```
    cd Dokumentasi-KRI
    cd klasifikasi_sampah
    ```
3. Install prasyarat
    ```
    python install -r requirements.txt 
   ```
4. Taruh model.pt di dalam folder klasifikasi_sampah
5. Run program
   ```
   python detect.py --weights model.pt --conf 0.7 --source 0 --device cpu
   ```
   atau
   ```
   python detect.py --weights model.pt --conf 0.7 --source 1 --device cpu
   ```
   
