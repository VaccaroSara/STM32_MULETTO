% 1. Legge i dati dal file 
dati = readmatrix(['sterzo120_60_0.8ms.txt']);

% 2. Separa le colonne
setpoint = dati(:, 1); % Prima colonna: comando dal radiocomando
imu_reale = dati(:, 2); % Seconda colonna: letta dal sensore BNO055

% 3. L'asse del tempo 
tempo = (0:length(setpoint)-1) * 0.01; 

% 4. Disegna il grafico
figure;
plot(tempo, setpoint, 'r--', 'LineWidth', 2); hold on;
plot(tempo, imu_reale, 'b-', 'LineWidth', 2);
grid on;
title('Analisi Risposta Dinamica FIR IMU: Setpoint vs IMU');
xlabel('Tempo (secondi)');
ylabel('Yaw Rate (rad/s)');
legend('Comando (Riferimento)', 'Realtà (IMU)', 'Location', 'best');
