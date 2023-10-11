from pyntcloud import PyntCloud

def main():
    cloud = PyntCloud.from_file("slam/test_pcd_out.pcd")
    print(cloud.points)

if __name__ == '__main__':
    main()