"""
Ejemplo de uso personalizado del simulador de frisbee
"""

from runge_kutta import FrisbeeSimulator
import numpy as np
import matplotlib.pyplot as plt

def ejemplo_basico():
    """Ejemplo básico de simulación"""
    print("=== EJEMPLO BÁSICO ===")
    
    # Crear simulador con parámetros por defecto
    sim = FrisbeeSimulator()
    
    # Simular con velocidades iniciales conocidas
    results = sim.simulate_flight(v_x0=20, v_y0=5, v_z0=3)
    
    # Mostrar resultados
    sim.print_results(results)
    
    return results

def ejemplo_parametros_personalizados():
    """Ejemplo con parámetros aerodinámicos personalizados"""
    print("\n=== EJEMPLO CON PARÁMETROS PERSONALIZADOS ===")
    
    # Frisbee con más arrastre y menos sustentación
    sim_pesado = FrisbeeSimulator(c_d=0.2, c_l=0.2, g=9.81)
    
    # Frisbee ligero con poca resistencia
    sim_ligero = FrisbeeSimulator(c_d=0.05, c_l=0.4, g=9.81)
    
    # Mismas condiciones iniciales para ambos
    v_x0, v_y0, v_z0 = 18, 8, 2
    
    results_pesado = sim_pesado.simulate_flight(v_x0, v_y0, v_z0)
    results_ligero = sim_ligero.simulate_flight(v_x0, v_y0, v_z0)
    
    print("Frisbee pesado (más arrastre):")
    sim_pesado.print_results(results_pesado)
    
    print("\nFrisbee ligero (menos arrastre):")
    sim_ligero.print_results(results_ligero)
    
    # Comparar trayectorias
    plt.figure(figsize=(12, 8))
    
    plt.subplot(2, 2, 1)
    plt.plot(results_pesado['x'], results_pesado['y'], 'r-', linewidth=2, label='Pesado')
    plt.plot(results_ligero['x'], results_ligero['y'], 'b-', linewidth=2, label='Ligero')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title('Vista desde arriba')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    plt.subplot(2, 2, 2)
    plt.plot(results_pesado['x'], results_pesado['z'], 'r-', linewidth=2, label='Pesado')
    plt.plot(results_ligero['x'], results_ligero['z'], 'b-', linewidth=2, label='Ligero')
    plt.xlabel('X (m)')
    plt.ylabel('Z (m)')
    plt.title('Vista lateral')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    plt.subplot(2, 2, 3)
    plt.plot(results_pesado['time'], results_pesado['z'], 'r-', linewidth=2, label='Pesado')
    plt.plot(results_ligero['time'], results_ligero['z'], 'b-', linewidth=2, label='Ligero')
    plt.xlabel('Tiempo (s)')
    plt.ylabel('Altura (m)')
    plt.title('Altura vs Tiempo')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    plt.subplot(2, 2, 4)
    speed_pesado = np.sqrt(results_pesado['vx']**2 + results_pesado['vy']**2)
    speed_ligero = np.sqrt(results_ligero['vx']**2 + results_ligero['vy']**2)
    plt.plot(results_pesado['time'], speed_pesado, 'r-', linewidth=2, label='Pesado')
    plt.plot(results_ligero['time'], speed_ligero, 'b-', linewidth=2, label='Ligero')
    plt.xlabel('Tiempo (s)')
    plt.ylabel('Velocidad horizontal (m/s)')
    plt.title('Velocidad vs Tiempo')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()

def ejemplo_multiples_lanzamientos():
    """Ejemplo de múltiples lanzamientos con diferentes ángulos"""
    print("\n=== EJEMPLO MÚLTIPLES LANZAMIENTOS ===")
    
    sim = FrisbeeSimulator()
    
    # Diferentes ángulos de lanzamiento
    velocidad_inicial = 20  # m/s
    angulos = [10, 20, 30, 40, 45]  # grados
    
    plt.figure(figsize=(12, 8))
    
    colores = ['red', 'blue', 'green', 'orange', 'purple']
    
    for i, angulo in enumerate(angulos):
        # Convertir ángulo a componentes de velocidad
        angulo_rad = np.radians(angulo)
        v_x0 = velocidad_inicial * np.cos(angulo_rad)
        v_y0 = 0  # sin componente lateral
        v_z0 = velocidad_inicial * np.sin(angulo_rad)
        
        results = sim.simulate_flight(v_x0, v_y0, v_z0)
        
        plt.subplot(2, 2, 1)
        plt.plot(results['x'], results['z'], color=colores[i], linewidth=2, 
                label=f'{angulo}°')
        
        plt.subplot(2, 2, 2)
        plt.plot(results['time'], results['z'], color=colores[i], linewidth=2, 
                label=f'{angulo}°')
        
        print(f"Ángulo {angulo}°: Distancia={results['distance']:.1f}m, "
              f"Tiempo={results['flight_time']:.2f}s, "
              f"Altura_max={results['max_height']:.2f}m")
    
    plt.subplot(2, 2, 1)
    plt.xlabel('Distancia horizontal (m)')
    plt.ylabel('Altura (m)')
    plt.title('Trayectorias por ángulo de lanzamiento')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    plt.subplot(2, 2, 2)
    plt.xlabel('Tiempo (s)')
    plt.ylabel('Altura (m)')
    plt.title('Altura vs Tiempo')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # Gráfico de alcance vs ángulo
    plt.subplot(2, 2, 3)
    distancias = []
    for angulo in angulos:
        angulo_rad = np.radians(angulo)
        v_x0 = velocidad_inicial * np.cos(angulo_rad)
        v_y0 = 0
        v_z0 = velocidad_inicial * np.sin(angulo_rad)
        results = sim.simulate_flight(v_x0, v_y0, v_z0)
        distancias.append(results['distance'])
    
    plt.plot(angulos, distancias, 'ko-', linewidth=2, markersize=8)
    plt.xlabel('Ángulo de lanzamiento (°)')
    plt.ylabel('Distancia alcanzada (m)')
    plt.title('Alcance vs Ángulo')
    plt.grid(True, alpha=0.3)
    
    # Tiempo de vuelo vs ángulo
    plt.subplot(2, 2, 4)
    tiempos = []
    for angulo in angulos:
        angulo_rad = np.radians(angulo)
        v_x0 = velocidad_inicial * np.cos(angulo_rad)
        v_y0 = 0
        v_z0 = velocidad_inicial * np.sin(angulo_rad)
        results = sim.simulate_flight(v_x0, v_y0, v_z0)
        tiempos.append(results['flight_time'])
    
    plt.plot(angulos, tiempos, 'ro-', linewidth=2, markersize=8)
    plt.xlabel('Ángulo de lanzamiento (°)')
    plt.ylabel('Tiempo de vuelo (s)')
    plt.title('Tiempo de vuelo vs Ángulo')
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()

def ejemplo_con_imu():
    """Ejemplo usando datos IMU del usuario"""
    print("\n=== EJEMPLO CON DATOS IMU ===")
    
    # Solicitar archivo CSV al usuario
    csv_file = input("Ingrese la ruta del archivo CSV con datos IMU: ").strip()
    
    # Crear simulador y cargar datos
    sim = FrisbeeSimulator()
    try:
        imu_data = sim.load_imu_data(csv_file)
        if imu_data is None:
            print("Error: No se pudieron cargar los datos IMU")
            return
    except Exception as e:
        print(f"Error al cargar el archivo: {e}")
        return
    
    # Calcular condiciones iniciales
    v_x0, v_y0, v_z0 = sim.calculate_initial_conditions(imu_data)
    
    # Simular vuelo
    results = sim.simulate_flight(v_x0, v_y0, v_z0)
    
    # Mostrar resultados y visualizar
    sim.print_results(results)
    sim.plot_simulation(results, imu_data)

if __name__ == "__main__":
    print("=== EJEMPLOS DEL SIMULADOR DE FRISBEE ===")
    print("Seleccione qué ejemplo ejecutar:")
    print("1. Ejemplo básico con velocidades conocidas")
    print("2. Comparación de parámetros aerodinámicos")
    print("3. Múltiples lanzamientos con diferentes ángulos")
    print("4. Análisis con datos IMU reales")
    print("5. Ejecutar todos los ejemplos")
    
    while True:
        try:
            opcion = int(input("\nIngrese su opción (1-5): "))
            if opcion == 1:
                ejemplo_basico()
                break
            elif opcion == 2:
                ejemplo_parametros_personalizados()
                break
            elif opcion == 3:
                ejemplo_multiples_lanzamientos()
                break
            elif opcion == 4:
                ejemplo_con_imu()
                break
            elif opcion == 5:
                ejemplo_basico()
                ejemplo_parametros_personalizados()
                ejemplo_multiples_lanzamientos()
                ejemplo_con_imu()
                break
            else:
                print("Opción inválida. Por favor ingrese un número del 1 al 5.")
        except ValueError:
            print("Por favor ingrese un número válido.")
