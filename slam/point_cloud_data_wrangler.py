# Data Wrangling Steps for PyntCloud Parser to load in dataset correctly

import re
def convert_dataset_to_pcd_file_format(str):
    str_buff = str[str.index("\""):].lstrip("\"").strip().rstrip("\"")
    pattern = r"\\n"
    matches = re.finditer(pattern, str_buff)
    newline_indicies = []
    newline_delimited_list = []
    left_index = 0

    for match in matches:
        newline_indicies.append((match.start(), match.end()-1))

    for indicies in newline_indicies:
        newline_delimited_list.append(str_buff[left_index: indicies[1]].rstrip('\\'))
        left_index = indicies[1] + 1
    return newline_delimited_list

def main():
    f = open("slam/out2.log", "r")
    buff = f.read()
    f.close()
    formatted_dataset = convert_dataset_to_pcd_file_format(buff)
    f = open("slam/test_pcd_out.pcd", "w")
    for str in formatted_dataset:
        f.write(str + "\n")
    f.close()

if __name__ == '__main__':
    main()