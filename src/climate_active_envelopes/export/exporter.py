'''
@author: kathrind
'''

import os

class Exporter:
    
    def __init__(self, file_name = '\\building_plan.txt', file_path = os.path.dirname(__file__) + '\\files'):
        self.file_name= file_path + file_name
        print(self.file_name)

    def delete_file(self):
        self.export_file = open(self.file_name, 'w')

    def open_file(self):
        self.export_file = open(self.file_name, 'a')

    def close_file(self):
        self.export_file.close()

    def export_pose(self, pose):
        self.open_file()
        self.write_pose(pose)
        self.close_file()

    def export_poses(self, poses):
        self.open_file()
        [self.write_pose(p) for p in poses]
        self.close_file()
    
    def export_poses_column(self, poses):
        self.open_file()
        [self.write_pose_column(p) for p in poses]
        self.close_file()
    
    def export_building_plan(self, building_plan):
        self.open_file()
        [self.write_line_building_plan(row) for row in building_plan]
        self.close_file()
    
    def write_line_building_plan(self, row):
        for item in row:
            self.export_file.write("%s " % item)
        self.export_file.write("\n")

    def write_pose(self, pose):
        for p in pose:
            self.export_file.write("%s " % p)
        self.export_file.write("\n")
    
    def write_pose_column(self, pose):
        self.export_file.write("%s " % "Frame")
        for p in pose:
            self.export_file.write("%s " % p)
        self.export_file.write("\n")

    def write_line(self, input):
        self.open_file()
        self.export_file.write("%s\n" % input)
        self.close_file()


if __name__ == '__main__':
    exporter = Exporter()
    exporter.delete_file()
    exporter.export_poses([[1,2,3,4,5,6,7], [1,2,3,4,5,6,7]])
