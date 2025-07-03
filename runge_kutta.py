import numpy as np
import matplotlib.pyplot as plt
import math

class FrisbeeSimulator:
    def __init__(self, c_d=0.1, c_l=0.3, g=9.81):
        """
        Inicializar el simulador de frisbee
        
        Parameters:
        c_d: coeficiente de arrastre
        c_l: coeficiente de sustentación
        g: aceleración de gravedad (m/s²)
        """
        self.c_d = c_d
        self.c_l = c_l
        self.g = g
        
    def load_imu_data(self, csv_file):
        """
        Cargar datos del archivo CSV con datos de IMU
        
        Parameters:
        csv_file: ruta al archivo CSV con columnas: timestamp, ax, ay, az, gx, gy, gz
        
        Nota: Las aceleraciones (ax, ay, az) deben estar en unidades G, se convertirán automáticamente a m/s²
        
        Returns:
        dict: datos procesados con aceleraciones en m/s²
        """
        try:
            import csv
            data = {
                'timestamp': [],
                'ax': [], 'ay': [], 'az': [],
                'gx': [], 'gy': [], 'gz': [],
                'time': []
            }
            
            with open(csv_file, 'r') as file:
                reader = csv.DictReader(file)
                for row in reader:
                    data['timestamp'].append(float(row['timestamp']))
                    data['ax'].append(float(row['ax']))
                    data['ay'].append(float(row['ay']))
                    data['az'].append(float(row['az']))
                    data['gx'].append(float(row['gx']))
                    data['gy'].append(float(row['gy']))
                    data['gz'].append(float(row['gz']))
            
            # Convertir a arrays numpy
            for key in data:
                data[key] = np.array(data[key])
            
            # Convertir aceleraciones de G a m/s² (multiplicar por 9.81)
            print("Convirtiendo aceleraciones de G a m/s²...")
            data['ax'] = data['ax'] * 9.81
            data['ay'] = data['ay'] * 9.81
            data['az'] = data['az'] * 9.81
            
            # Convertir timestamp de milisegundos a segundos
            data['time'] = data['timestamp'] / 1000.0
            # Resetear tiempo para que empiece en 0
            data['time'] = data['time'] - data['time'][0]
            
            return data
        except Exception as e:
            print(f"Error al cargar el archivo CSV: {e}")
            return None
    
    def calculate_initial_conditions(self, imu_data, launch_window=0.5):
        """
        Calcular condiciones iniciales del lanzamiento basado en datos IMU
        
        Parameters:
        imu_data: DataFrame con datos de IMU
        launch_window: ventana de tiempo (segundos) para detectar el lanzamiento
        
        Returns:
        tuple: (v_x0, v_y0, v_z0, real_flight_time) velocidades iniciales y tiempo de vuelo real
        """
        # Calcular tiempo de vuelo real desde los datos IMU
        real_flight_time = (imu_data['timestamp'][-1] - imu_data['timestamp'][0]) / 1000.0  # Convertir ms a segundos
        print(f"Tiempo de vuelo real (desde datos IMU): {real_flight_time:.3f} segundos ({imu_data['timestamp'][-1] - imu_data['timestamp'][0]:.0f} ms)")
        
        # Detectar el momento de lanzamiento (pico de aceleración)
        acc_magnitude = np.sqrt(imu_data['ax']**2 + imu_data['ay']**2 + imu_data['az']**2)
        launch_idx = np.argmax(acc_magnitude)
        
        # Calcular velocidad inicial integrando aceleración durante la ventana de lanzamiento
        launch_time = imu_data['time'][launch_idx]
        window_mask = (imu_data['time'] >= launch_time) & (imu_data['time'] <= launch_time + launch_window)
        
        if np.sum(window_mask) < 2:
            # Si no hay suficientes datos, usar valores por defecto
            print("Advertencia: Datos insuficientes para calcular velocidad inicial. Usando valores por defecto.")
            return 15.0, 8.0, 0.0, real_flight_time  # velocidades típicas de frisbee + tiempo real
        
        # Integrar aceleración para obtener velocidad
        time_window = imu_data['time'][window_mask]
        ax_window = imu_data['ax'][window_mask]
        ay_window = imu_data['ay'][window_mask]
        az_window = imu_data['az'][window_mask]
        
        # Velocidad inicial (asumiendo que parte del reposo)
        v_x = np.trapezoid(ax_window, time_window)
        v_y = np.trapezoid(ay_window, time_window)
        v_z = np.trapezoid(az_window, time_window)
        
        # Filtrar valores extremos
        v_x = max(-50, min(50, v_x))
        v_y = max(-50, min(50, v_y))
        v_z = max(-20, min(20, v_z))
        
        print(f"Velocidades iniciales calculadas: vx={v_x:.2f}, vy={v_y:.2f}, vz={v_z:.2f} m/s")
        return v_x, v_y, v_z, real_flight_time
    
    def frisbee_dynamics(self, t, state):
        """
        Ecuaciones diferenciales del movimiento del frisbee
        
        Parameters:
        t: tiempo
        state: [x, y, z, vx, vy, vz]
        
        Returns:
        list: derivadas [dx/dt, dy/dt, dz/dt, dvx/dt, dvy/dt, dvz/dt]
        """
        x, y, z, vx, vy, vz = state
        
        # Ecuaciones diferenciales del frisbee
        dxdt = vx
        dydt = vy
        dzdt = vz
        
        # Ecuaciones de velocidad con aerodinámica
        dvxdt = -self.c_d * vx + self.c_l * vy
        dvydt = -self.c_l * vx - self.c_d * vy
        dvzdt = -self.g - self.c_d * vz  # gravedad y arrastre vertical
        
        return [dxdt, dydt, dzdt, dvxdt, dvydt, dvzdt]
    
    def runge_kutta_4(self, f, y0, t_span, dt=0.01, stop_on_ground=True):
        """
        Implementación del método Runge-Kutta de 4to orden
        
        Parameters:
        f: función que define el sistema de ecuaciones diferenciales
        y0: condiciones iniciales
        t_span: tupla (t_inicial, t_final)
        dt: paso de tiempo
        stop_on_ground: si detener cuando z < 0 (por defecto True)
        
        Returns:
        tuple: (tiempo, solución)
        """
        t_start, t_end = t_span
        t = np.arange(t_start, t_end + dt, dt)
        y = np.zeros((len(t), len(y0)))
        y[0] = y0
        
        for i in range(len(t) - 1):
            h = t[i+1] - t[i]
            
            k1 = np.array(f(t[i], y[i]))
            k2 = np.array(f(t[i] + h/2, y[i] + h*k1/2))
            k3 = np.array(f(t[i] + h/2, y[i] + h*k2/2))
            k4 = np.array(f(t[i] + h, y[i] + h*k3))
            
            y[i+1] = y[i] + h/6 * (k1 + 2*k2 + 2*k3 + k4)
            
            # Detener si el frisbee toca el suelo (solo si stop_on_ground=True)
            if stop_on_ground and y[i+1][2] < 0:  # z < 0
                y = y[:i+2]
                t = t[:i+2]
                y[-1][2] = 0  # Ajustar z final a 0
                break
        
        return t, y
    
    def simulate_flight(self, v_x0, v_y0, v_z0=0, x0=0, y0=0, z0=1.5, max_time=10, real_flight_time=None):
        """
        Simular el vuelo del frisbee
        
        Parameters:
        v_x0, v_y0, v_z0: velocidades iniciales
        x0, y0, z0: posición inicial
        max_time: tiempo máximo de simulación
        real_flight_time: tiempo real de vuelo desde datos IMU (opcional)
        
        Returns:
        dict: resultados de la simulación
        """
        # Condiciones iniciales
        initial_state = [x0, y0, z0, v_x0, v_y0, v_z0]
        
        # Si tenemos tiempo real, usarlo como tiempo de simulación
        if real_flight_time is not None:
            sim_time = real_flight_time
            print(f"Simulando por tiempo real de vuelo: {real_flight_time:.3f} segundos")
        else:
            sim_time = max_time
        
        # Resolver usando Runge-Kutta de 4to orden
        t, solution = self.runge_kutta_4(self.frisbee_dynamics, initial_state, (0, sim_time))
        
        # Extraer resultados
        x = solution[:, 0]
        y = solution[:, 1]
        z = solution[:, 2]
        vx = solution[:, 3]
        vy = solution[:, 4]
        vz = solution[:, 5]
        
        # Calcular métricas
        simulated_flight_time = t[-1] - t[0]  # Tiempo final - Tiempo inicial de simulación
        
        # Usar tiempo real si está disponible, sino usar el simulado
        if real_flight_time is not None:
            flight_time = real_flight_time
            print(f"Tiempo de vuelo usado: {real_flight_time:.3f} segundos")
            print(f"Altura final simulada: {z[-1]:.3f} metros")
        else:
            flight_time = simulated_flight_time
        
        max_distance = np.sqrt(x[-1]**2 + y[-1]**2)
        max_height = np.max(z)
        
        results = {
            'time': t,
            'x': x, 'y': y, 'z': z,
            'vx': vx, 'vy': vy, 'vz': vz,
            'flight_time': flight_time,
            'simulated_flight_time': simulated_flight_time,
            'distance': max_distance,
            'max_height': max_height
        }
        
        return results
    
    def plot_simulation(self, results, imu_data=None):
        """
        Crear visualizaciones de la simulación
        
        Parameters:
        results: resultados de la simulación
        imu_data: datos IMU opcionales para mostrar
        """
        fig = plt.figure(figsize=(15, 10))
        
        # Vista desde arriba (X-Y)
        ax1 = plt.subplot(2, 3, 1)
        plt.plot(results['x'], results['y'], 'b-', linewidth=2, label='Trayectoria')
        plt.plot(results['x'][0], results['y'][0], 'go', markersize=8, label='Inicio')
        plt.plot(results['x'][-1], results['y'][-1], 'ro', markersize=8, label='Final')
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.title('Vista desde arriba (X-Y)')
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.axis('equal')
        
        # Vista lateral (X-Z)
        ax2 = plt.subplot(2, 3, 2)
        plt.plot(results['x'], results['z'], 'r-', linewidth=2, label='Trayectoria')
        plt.plot(results['x'][0], results['z'][0], 'go', markersize=8, label='Inicio')
        plt.plot(results['x'][-1], results['z'][-1], 'ro', markersize=8, label='Final')
        plt.xlabel('X (m)')
        plt.ylabel('Z (m)')
        plt.title('Vista lateral (X-Z)')
        plt.grid(True, alpha=0.3)
        plt.legend()
        
        # Vista lateral (Y-Z)
        ax3 = plt.subplot(2, 3, 3)
        plt.plot(results['y'], results['z'], 'g-', linewidth=2, label='Trayectoria')
        plt.plot(results['y'][0], results['z'][0], 'go', markersize=8, label='Inicio')
        plt.plot(results['y'][-1], results['z'][-1], 'ro', markersize=8, label='Final')
        plt.xlabel('Y (m)')
        plt.ylabel('Z (m)')
        plt.title('Vista lateral (Y-Z)')
        plt.grid(True, alpha=0.3)
        plt.legend()
        
        # Velocidades vs tiempo
        ax4 = plt.subplot(2, 3, 4)
        plt.plot(results['time'], results['vx'], 'r-', label='vx')
        plt.plot(results['time'], results['vy'], 'g-', label='vy')
        plt.plot(results['time'], results['vz'], 'b-', label='vz')
        plt.xlabel('Tiempo (s)')
        plt.ylabel('Velocidad (m/s)')
        plt.title('Velocidades vs Tiempo')
        plt.grid(True, alpha=0.3)
        plt.legend()
        
        # Altura vs tiempo
        ax5 = plt.subplot(2, 3, 5)
        plt.plot(results['time'], results['z'], 'b-', linewidth=2)
        plt.xlabel('Tiempo (s)')
        plt.ylabel('Altura (m)')
        plt.title('Altura vs Tiempo')
        plt.grid(True, alpha=0.3)
        
        # Datos IMU (si están disponibles)
        if imu_data is not None:
            ax6 = plt.subplot(2, 3, 6)
            plt.plot(imu_data['time'], imu_data['ax'], 'r-', alpha=0.7, label='ax')
            plt.plot(imu_data['time'], imu_data['ay'], 'g-', alpha=0.7, label='ay')
            plt.plot(imu_data['time'], imu_data['az'], 'b-', alpha=0.7, label='az')
            plt.xlabel('Tiempo (s)')
            plt.ylabel('Aceleración (m/s²)')
            plt.title('Datos IMU - Aceleración')
            plt.grid(True, alpha=0.3)
            plt.legend()
        
        plt.tight_layout()
        plt.show()
    
    def print_results(self, results):
        """
        Imprimir resumen de resultados
        """
        print("\n" + "="*50)
        print("RESULTADOS DE LA SIMULACIÓN")
        print("="*50)
        print(f"Tiempo de vuelo (datos IMU): {results['flight_time']:.3f} segundos")
        if 'simulated_flight_time' in results:
            print(f"Tiempo de simulación: {results['simulated_flight_time']:.3f} segundos")
        print(f"Distancia recorrida: {results['distance']:.2f} metros")
        print(f"Altura máxima: {results['max_height']:.2f} metros")
        print(f"Posición final: X={results['x'][-1]:.2f}m, Y={results['y'][-1]:.2f}m")
        print(f"Velocidad inicial: {np.sqrt(results['vx'][0]**2 + results['vy'][0]**2 + results['vz'][0]**2):.2f} m/s")
        print("="*50)

def create_sample_csv(filename="sample_imu_data.csv"):
    """
    Crear un archivo CSV de ejemplo con datos simulados de IMU
    Las aceleraciones se generan en unidades G (para simular datos reales de IMU)
    """
    # Simular datos de lanzamiento de frisbee
    time_ms = np.arange(0, 3000, 10)  # 3 segundos de datos, cada 10ms
    n_points = len(time_ms)
    
    # Simular aceleración durante el lanzamiento
    ax = np.zeros(n_points)
    ay = np.zeros(n_points)
    az = np.zeros(n_points)
    
    # Fase de lanzamiento (primeros 0.5 segundos)
    launch_duration = 50  # 500ms
    ax[:launch_duration] = 1.5 * np.exp(-np.arange(launch_duration) / 20) + np.random.normal(0, 0.05, launch_duration)  # En G
    ay[:launch_duration] = 0.8 * np.exp(-np.arange(launch_duration) / 25) + np.random.normal(0, 0.03, launch_duration)  # En G
    az[:launch_duration] = -1.0 + 0.5 * np.exp(-np.arange(launch_duration) / 15) + np.random.normal(0, 0.02, launch_duration)  # En G
    
    # Fase de vuelo libre
    ax[launch_duration:] = np.random.normal(0, 0.01, n_points - launch_duration)  # En G
    ay[launch_duration:] = np.random.normal(0, 0.01, n_points - launch_duration)  # En G
    az[launch_duration:] = np.random.normal(-1.0, 0.02, n_points - launch_duration)  # En G (aproximadamente -1G por gravedad)
    
    # Simular velocidad angular (giroscopio)
    gx = np.random.normal(0, 50, n_points)  # Rotación del frisbee
    gy = np.random.normal(0, 20, n_points)
    gz = np.random.normal(200, 30, n_points)  # Spin principal del frisbee
    
    # Crear DataFrame
    import csv
    
    with open(filename, 'w', newline='') as file:
        writer = csv.writer(file)
        # Escribir encabezados
        writer.writerow(['timestamp', 'ax', 'ay', 'az', 'gx', 'gy', 'gz'])
        # Escribir datos
        for i in range(n_points):
            writer.writerow([time_ms[i], ax[i], ay[i], az[i], gx[i], gy[i], gz[i]])
    print(f"Archivo de ejemplo creado: {filename}")
    return filename

def main():
    """
    Función principal para ejecutar la simulación
    """
    print("=== SIMULADOR DE LANZAMIENTO DE FRISBEE ===")
    print("Método Runge-Kutta de 4to orden")
    print("="*50)
    
    # Solicitar archivo CSV al usuario
    while True:
        csv_file = input("Ingrese la ruta del archivo CSV con datos IMU: ").strip()
        if csv_file == "":
            print("Error: Debe ingresar una ruta de archivo.")
            continue
        
        # Crear simulador
        simulator = FrisbeeSimulator(c_d=0.1, c_l=0.3, g=9.81)
        
        # Intentar cargar el archivo
        try:
            print(f"Cargando archivo: {csv_file}")
            imu_data = simulator.load_imu_data(csv_file)
            if imu_data is None:
                raise FileNotFoundError("No se pudieron procesar los datos del archivo")
            print("✓ Archivo cargado exitosamente")
            break
        except FileNotFoundError:
            print(f"✗ Error: No se encontró el archivo '{csv_file}'")
            print("Por favor, verifique la ruta e intente nuevamente.")
        except Exception as e:
            print(f"✗ Error al cargar el archivo: {e}")
            print("Verifique que el archivo tenga el formato correcto:")
            print("timestamp,ax,ay,az,gx,gy,gz")
    
    # Preguntar si quiere personalizar parámetros
    print("\n¿Desea personalizar los parámetros aerodinámicos? (s/n): ", end="")
    personalizar = input().strip().lower()
    
    if personalizar in ['s', 'si', 'sí', 'y', 'yes']:
        try:
            print("Parámetros actuales: c_d=0.1, c_l=0.3, g=9.81")
            c_d = float(input("Coeficiente de arrastre (c_d) [0.1]: ") or 0.1)
            c_l = float(input("Coeficiente de sustentación (c_l) [0.3]: ") or 0.3)
            g = float(input("Aceleración de gravedad (g) [9.81]: ") or 9.81)
            simulator = FrisbeeSimulator(c_d=c_d, c_l=c_l, g=g)
            print(f"✓ Parámetros actualizados: c_d={c_d}, c_l={c_l}, g={g}")
        except ValueError:
            print("✗ Error en los parámetros. Usando valores por defecto.")
    
    # Calcular condiciones iniciales
    print("\nAnalizando datos IMU...")
    v_x0, v_y0, v_z0, real_flight_time = simulator.calculate_initial_conditions(imu_data)
    
    # Simular vuelo
    print("Ejecutando simulación con Runge-Kutta de 4to orden...")
    results = simulator.simulate_flight(v_x0, v_y0, v_z0, real_flight_time=real_flight_time)
    
    # Mostrar resultados
    simulator.print_results(results)
    
    # Preguntar si quiere ver las gráficas
    print("\n¿Desea mostrar las visualizaciones? (s/n): ", end="")
    mostrar_graficas = input().strip().lower()
    
    if mostrar_graficas in ['s', 'si', 'sí', 'y', 'yes']:
        print("Generando visualizaciones...")
        simulator.plot_simulation(results, imu_data)
    
    print("\n¡Simulación completada!")

if __name__ == "__main__":
    main()