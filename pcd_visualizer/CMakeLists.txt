cmake_minimum_required(VERSION 3.10)
project(PCLViewer)

# C++の標準バージョンを設定
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# PCLライブラリの設定
find_package(PCL 1.8 REQUIRED)

find_package(X11 REQUIRED)

# コンパイラに警告を表示させるオプションを追加
if (CMAKE_COMPILER_IS_GNUCXX)
    add_compile_options(-Wall -Wextra -pedantic)
endif()

# インクルードディレクトリを設定
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 実行ファイルを作成
add_executable(pcl_viewer src/main.cpp)

# 必要なPCLライブラリをリンク
target_link_libraries(pcl_viewer ${PCL_LIBRARIES} ${X11_LIBRARIES})
