# Data Wrangling Steps for PyntCloud Parser to load in dataset correctly:
# (1) strip 'point_cloud_pcd_chunk: "' in heading and trailing '"'
# (2) remove '\n' with actual line break
# https://stackoverflow.com/questions/42965689/replacing-a-text-with-n-in-it-with-a-real-n-output
# https://stackoverflow.com/questions/54586164/how-to-replace-all-instances-of-n-in-a-string-with-linebreaks

def convert_dataset_to_pcd_file_format(str):
    pass


def main():
    # load in string from file = input_dataset
    # formatted_dataset = convert_dataset_to_pcd_file_format(input_dataset)
    # write formatted dataset to new file
    pass

if __name__ == '__main__':
    main()