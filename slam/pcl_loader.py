from pyntcloud import PyntCloud

def main():
    cloud = PyntCloud.from_file("slam/pcd_map_clean.pcd")
    print(cloud.points)

if __name__ == '__main__':
    main()