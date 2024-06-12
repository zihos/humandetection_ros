import pandas as pd

def calculate_statistics_from_csv(file_path, column_name):
    # CSV 파일 읽기
    df = pd.read_csv(file_path)
    
    # "usr usage" 컬럼 값 추출
    column_values = df[column_name]
    
    # 평균과 표준 편차 계산
    mean_value = column_values.mean()
    std_dev = column_values.std()
    
    return mean_value, std_dev

# 예제 파일 경로와 컬럼 이름
file_path = '/home/zio/humandetection_ros/results/mpstat_output_camera.csv'
column_name = 'usr usage'

# 함수 호출
mean_value, std_dev = calculate_statistics_from_csv(file_path, column_name)

print(f"Mean of {column_name}: {mean_value:.2f}")
print(f"Standard Deviation of {column_name}: {std_dev:.2f}")
