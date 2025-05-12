import pygame, math
import paho.mqtt.client as mqtt
pygame.init()


LIDAR_RESOLUTION = 100
VISUALIZATION_RESOLUTION = LIDAR_RESOLUTION


def generate_graphical_position(lines_number) :
    angle = 360/lines_number
    lines = []

    for point in range(lines_number) :
        lines.append.
