include_directories(./include)

AUX_SOURCE_DIRECTORY(./src AUTOAIM_SRCS)
ADD_LIBRARY(AutoAim ${AUTOAIM_SRCS})
find_package(TensorflowCC REQUIRED)
target_link_libraries(AutoAim BaseAim)
target_link_libraries(AutoAim TensorflowCC::Static)