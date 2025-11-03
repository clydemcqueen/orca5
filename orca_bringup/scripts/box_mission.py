#!/usr/bin/env python3

"""
Create a simple box mission.

Usage:
./scripts/box_mission.py > missions/box_mission.txt
"""

from geopy import distance

# The sub origin, this should match the call to ardusub in sitl_dds_udp.launch.py
ORIGIN = (47.6302, -122.3982391)

# Box sizes in meters
BOX_SIZE = (8.0, 12.0)


def compute_location(lat, lon, dist_n, dist_e):
    """
    Compute the location given a distance in meters north and east.

    Args:
        lat (float): Starting latitude in degrees
        lon (float): Starting longitude in degrees
        dist_n (float): Distance north in meters (positive is north, negative is south)
        dist_e (float): Distance east in meters (positive is east, negative is west)

    Returns:
        tuple: (new_lat, new_lon) in degrees
    """
    # First move north/south
    if dist_n != 0:
        point = distance.distance(meters=abs(dist_n)).destination(
            (lat, lon),
            bearing=0 if dist_n > 0 else 180
        )
        lat, lon = point.latitude, point.longitude

    # Then move east/west
    if dist_e != 0:
        point = distance.distance(meters=abs(dist_e)).destination(
            (lat, lon),
            bearing=90 if dist_e > 0 else 270
        )
        lat, lon = point.latitude, point.longitude

    return lat, lon


def main():
    """Given a lat and lon, create a simple box mission that moves in a square."""

    lat1, lon1 = compute_location(ORIGIN[0], ORIGIN[1], -BOX_SIZE[0] / 2, -BOX_SIZE[1] / 2)
    lat2, lon2 = compute_location(lat1, lon1, BOX_SIZE[0], BOX_SIZE[1])

    # MAVLink frame 3 (relative altitude)
    frame = 3

    # Altitude in meters
    alt = -1.0

    # Write the mission. Item 0 is required but ignored.
    print('QGC WPL 110')
    print(f'0 0 {frame} 16 0 0 0 0 {lat1} {lon1} {alt} 1')
    print(f'1 0 {frame} 16 0 0 0 0 {lat2} {lon1} {alt} 1')
    print(f'2 0 {frame} 16 0 0 0 0 {lat2} {lon2} {alt} 1')
    print(f'3 0 {frame} 16 0 0 0 0 {lat1} {lon2} {alt} 1')
    print(f'4 0 {frame} 16 0 0 0 0 {lat1} {lon1} {alt} 1')
    print(f'5 0 {frame} 16 0 0 0 0 {lat2} {lon1} {alt} 1')


if __name__ == '__main__':
    main()
