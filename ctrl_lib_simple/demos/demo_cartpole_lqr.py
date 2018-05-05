from ctrl_lib_simple.envs.cartpole import CartPoleEnv
from ctrl_lib_simple.controllers.lqr import LQR
import argparse

if __name__ == '__main__':
    
    parser = argparse.ArgumentParser(description='Demo for controlling an inverted pendulum on a 1D cart using LQR')

    parser.add_argument("--cart_mass", default = 5.0, type = float, help = "Mass of cart")
    parser.add_argument("--pole_length", default = 1.0, type = float, help = "Length of pendulum")
    parser.add_argument("--pendulum_mass", default = 1.0, type = float, help = "Mass of pendulum bob")
    parser.add_argument("--cart_start_pos", default = 0.2, type = float, help = "Starting position of cart")
    parser.add_argument("--pendulum_start_angle", default = 1.0, type = float, help = "Starting orientation of pendulum")
    parser.add_argument("--g", default = 9.81, type = float, help = "Acceleration due to gravity")

    parser.add_argument("--desired_cart_pos", default = 0.3, type = float, help = "Desired final position of cart")
    parser.add_argument("--desired_pendulum_angle", default = 0., type = float, help = "Desired final orientation (in rad) of pendulum (vertically up is 0)")

    parser.add_argument("-P","--plot", default = 0, type = int, help = "Whether or not to plot the graphs for forces, error etc.")


    args = parser.parse_args()

    env = CartPoleEnv(cart_mass = args.cart_mass, pole_length = args.pole_length, pendulum_mass = args.pendulum_mass, cart_start_pos = args.cart_start_pos, pendulum_start_angle = args.pendulum_start_angle, g = args.g)

    env.control_cartpole(control_type = 'lqr', desired_cart_pos = args.desired_cart_pos, desired_pendulum_angle = args.desired_pendulum_angle, plot = bool(args.plot))