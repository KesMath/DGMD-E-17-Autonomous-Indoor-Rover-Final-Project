from pyntcloud import PyntCloud
import matplotlib.pyplot as plt 


def main():
    cloud = PyntCloud.from_file("slam/map.pcd")
    x_arr = cloud.points.x
    y_arr = cloud.points.y

    plt.scatter(x_arr, y_arr)
    plt.xlabel('Horizontal Distance') 
    plt.ylabel('Vertical Distance')
    plt.title('LiDAR Snapshot of Enclosure') 
    plt.show() 

if __name__ == '__main__':
    main()