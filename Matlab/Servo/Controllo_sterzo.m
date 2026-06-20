% 1. Legge i dati dal file (readmatrix ignora automaticamente eventuali testi sparsi)
dati = readmatrix(['sterzo120_60_0.8ms.txt']);

% 2. Separa le colonne
setpoint = dati(:, 1); % Prima colonna: il tuo comando dal radiocomando
imu_reale = dati(:, 2); % Seconda colonna: la realtà letta dal sensore BNO055

% 3. Crea l'asse del tempo (sappiamo che il ciclo gira ogni 10 millisecondi)
tempo = (0:length(setpoint)-1) * 0.01; 

% 4. Disegna il grafico
figure;
plot(tempo, setpoint, 'r--', 'LineWidth', 2); hold on;
plot(tempo, imu_reale, 'b-', 'LineWidth', 2);

% 5. Estetica da veri Ingegneri
grid on;
title('Analisi Risposta Dinamica FIR IMU: Setpoint vs IMU');
xlabel('Tempo (secondi)');
ylabel('Yaw Rate (rad/s)');
legend('Comando (Riferimento)', 'Realtà (IMU)', 'Location', 'best');

% 1. Legge i dati dal file (readmatrix ignora automaticamente eventuali testi sparsi)
dati = readmatrix(['sterzo120_60_0.9ms.txt']);

% 2. Separa le colonne
setpoint = dati(:, 1); % Prima colonna: il tuo comando dal radiocomando
imu_reale = dati(:, 2); % Seconda colonna: la realtà letta dal sensore BNO055

% 3. Crea l'asse del tempo (sappiamo che il ciclo gira ogni 10 millisecondi)
tempo = (0:length(setpoint)-1) * 0.01; 

% 4. Disegna il grafico
figure;
plot(tempo, setpoint, 'r--', 'LineWidth', 2); hold on;
plot(tempo, imu_reale, 'b-', 'LineWidth', 2);

% 5. Estetica da veri Ingegneri
grid on;
title('Analisi Risposta Dinamica FIR IMU: Setpoint vs IMU');
xlabel('Tempo (secondi)');
ylabel('Yaw Rate (rad/s)');
legend('Comando (Riferimento)', 'Realtà (IMU)', 'Location', 'best');

% 1. Legge i dati dal file (readmatrix ignora automaticamente eventuali testi sparsi)
dati = readmatrix(['sterzo120_60_FIR3_IMU']);

% 2. Separa le colonne
setpoint = dati(:, 1); % Prima colonna: il tuo comando dal radiocomando
imu_reale = dati(:, 2); % Seconda colonna: la realtà letta dal sensore BNO055

% 3. Crea l'asse del tempo (sappiamo che il ciclo gira ogni 10 millisecondi)
tempo = (0:length(setpoint)-1) * 0.01; 

% 4. Disegna il grafico
figure;
plot(tempo, setpoint, 'r--', 'LineWidth', 2); hold on;
plot(tempo, imu_reale, 'b-', 'LineWidth', 2);

% 5. Estetica da veri Ingegneri
grid on;
title('Analisi Risposta Dinamica FIR6: Setpoint vs IMU');
xlabel('Tempo (secondi)');
ylabel('Yaw Rate (rad/s)');
legend('Comando (Riferimento)', 'Realtà (IMU)', 'Location', 'best');

% 1. Legge i dati dal file (readmatrix ignora automaticamente eventuali testi sparsi)
dati = readmatrix(['sterzo120_60_0.6radss.txt']);

% 2. Separa le colonne
setpoint = dati(:, 1); % Prima colonna: il tuo comando dal radiocomando
imu_reale = dati(:, 2); % Seconda colonna: la realtà letta dal sensore BNO055

% 3. Crea l'asse del tempo (sappiamo che il ciclo gira ogni 10 millisecondi)
tempo = (0:length(setpoint)-1) * 0.01; 

% 4. Disegna il grafico
figure;
plot(tempo, setpoint, 'r--', 'LineWidth', 2); hold on;
plot(tempo, imu_reale, 'b-', 'LineWidth', 2);

% 5. Estetica da veri Ingegneri
grid on;
title('Analisi Risposta Dinamica FIR6: Setpoint vs IMU');
xlabel('Tempo (secondi)');
ylabel('Yaw Rate (rad/s)');
legend('Comando (Riferimento)', 'Realtà (IMU)', 'Location', 'best');

% 1. Legge i dati dal file (readmatrix ignora automaticamente eventuali testi sparsi)
dati = readmatrix(['sterzo120_60_1.4radss.txt']);

% 2. Separa le colonne
setpoint = dati(:, 1); % Prima colonna: il tuo comando dal radiocomando
imu_reale = dati(:, 2); % Seconda colonna: la realtà letta dal sensore BNO055

% 3. Crea l'asse del tempo (sappiamo che il ciclo gira ogni 10 millisecondi)
tempo = (0:length(setpoint)-1) * 0.01; 

% 4. Disegna il grafico
figure;
plot(tempo, setpoint, 'r--', 'LineWidth', 2); hold on;
plot(tempo, imu_reale, 'b-', 'LineWidth', 2);

% 5. Estetica da veri Ingegneri
grid on;
title('Analisi Risposta Dinamica FIR6: Setpoint vs IMU');
xlabel('Tempo (secondi)');
ylabel('Yaw Rate (rad/s)');
legend('Comando (Riferimento)', 'Realtà (IMU)', 'Location', 'best');


