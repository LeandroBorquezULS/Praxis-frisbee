import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math

G = 9.81

class SimuladorFrisbee:
    def __init__(self, cd, cl, G, timestamps, ax, ay):
        self.cd = cd
        self.cl = cl
        self.G = G
        self.timestamps = timestamps
        self.ax = ax
        self.ay = ay

        self.positions_x = []
        self.positions_y = []
        self.velocities_x = []
        self.velocities_y = []

    def simular(self, vx0=0, vy0=0, x0=0, y0=0):
        vx, vy = vx0, vy0
        x, y = x0, y0

        self.positions_x = [x]
        self.positions_y = [y]
        self.velocities_x = [vx]
        self.velocities_y = [vy]

        for i in range(1, len(self.timestamps)):
            dt = (self.timestamps[i] - self.timestamps[i-1]) / 1000.0

            # Calcular aceleración total con Pitágoras
            a = math.sqrt(self.ax[i]**2 + self.ay[i]**2)

            # Puedes usar 'a' en vez de ax y ay, por ejemplo:
            # Aquí se reparte la aceleración en la dirección actual de movimiento
            if vx == 0 and vy == 0:
                ax_unit, ay_unit = 1, 0  # Por defecto hacia X si está en reposo
            else:
                norm = math.sqrt(vx**2 + vy**2)
                ax_unit = vx / norm
                ay_unit = vy / norm

            ax_eff = a * ax_unit
            ay_eff = a * ay_unit

            dvx = -self.cd * vx + self.cl * vy + ax_eff
            vx += dvx * dt

            dvy = -self.G + self.cl * vx - self.cd * vy + ay_eff
            vy += dvy * dt

            x += vx * dt
            y += vy * dt

            if y < 0:
                y = 0
                self.positions_x.append(x)
                self.positions_y.append(y)
                self.velocities_x.append(vx)
                self.velocities_y.append(vy)
                break

            self.positions_x.append(x)
            self.positions_y.append(y)
            self.velocities_x.append(vx)
            self.velocities_y.append(vy)

    def animar(self):
        fig, ax = plt.subplots()
        ax.set_title('Animación del movimiento del frisbee')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.grid()
        recorrido, = ax.plot([], [], 'b-', lw=2, label='Recorrido')
        punto, = ax.plot([], [], 'ro', label='Frisbee')
        ax.set_xlim(min(self.positions_x) - 1, max(self.positions_x) + 1)
        ax.set_ylim(min(self.positions_y) - 1, max(self.positions_y) + 1)
        ax.legend()

        def update(frame):
            recorrido.set_data(self.positions_x[:frame+1], self.positions_y[:frame+1])
            punto.set_data([self.positions_x[frame]], [self.positions_y[frame]])
            return recorrido, punto

        ani = animation.FuncAnimation(
            fig, update, frames=len(self.positions_x), interval=300, blit=True, repeat=False
        )
        plt.show()

def main():
    cd = 0.08
    cl = 0.15
    timestamps = [i*100 for i in range(21)]  # 0, 100, ..., 2000 ms
    ax = [6 - 0.1*i for i in range(21)]      # Disminuye el impulso poco a poco
    ay = [4 - 0.08*i for i in range(21)]   # Sustentación que va bajando

    sim = SimuladorFrisbee(cd, cl, G, timestamps, ax, ay)
    sim.simular()
    sim.animar()  # Solo la animación

if __name__ == "__main__":
    main()