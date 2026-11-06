1. Tener un fork del repositorio de PX4-Autopilot
2. Hacer los correspondientes cambios en NUESTRO fork, no en el original
3. Actualizar el symlink (git submodule) del repo PX4-Autopilot dentro de AutonomousDrone para que apunte a nuestro fork.
4. Para actualizar la dependencia, cada vez que esta tenga cambios: `git submodule update --remote`
