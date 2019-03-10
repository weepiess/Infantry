include_directories(./include)

AUX_SOURCE_DIRECTORY(./src AUTOAIM_SRCS)
find_package(TensorflowCC REQUIRED)
ADD_LIBRARY(AutoAim ${AUTOAIM_SRCS})
target_link_libraries(AutoAim BaseAim)
target_link_libraries(AutoAim TensorflowCC::Static)
