cmake_minimum_required(VERSION 3.20)
project(AllegroLidar)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------
# SFML 2.6.1 konumunu belirt (lib/cmake/SFML klasörünü göster!)
# ---------------------------------------------------------------
set(SFML_DIR "C:/Libraries/sfml/SFML-2.6.1/lib/cmake/SFML")

# ---------------------------------------------------------------
# Gerekli SFML bileşenlerini bul
# ---------------------------------------------------------------
find_package(SFML 2.6.1 COMPONENTS graphics window system REQUIRED)

# ---------------------------------------------------------------
# libcurl yollarını belirt
# ---------------------------------------------------------------
set(CURL_INCLUDE_DIR "C:/Libraries/sfml/libcurl/include")
set(CURL_LIBRARY "C:/Libraries/sfml/libcurl/lib/libcurl.a")

include_directories(${CURL_INCLUDE_DIR})

# ---------------------------------------------------------------
# Çalıştırılabilir dosya
# ---------------------------------------------------------------
add_executable(SFML_Lidar main.cpp)

# ---------------------------------------------------------------
# SFML ve libcurl kütüphanelerini bağla
# ---------------------------------------------------------------
target_link_libraries(SFML_Lidar PRIVATE
        sfml-graphics
        sfml-window
        sfml-system
        ${CURL_LIBRARY}
)

# ---------------------------------------------------------------
# SFML DLL dosyalarını otomatik olarak kopyala
# ---------------------------------------------------------------
set(SFML_DLL_DIR "C:/Libraries/sfml/SFML-2.6.1/bin")
file(GLOB SFML_DLLS "${SFML_DLL_DIR}/*.dll")

add_custom_command(TARGET SFML_Lidar POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${SFML_DLLS}
        $<TARGET_FILE_DIR:SFML_Lidar>
        COMMENT "SFML DLL dosyaları kopyalanıyor..."
)

# ---------------------------------------------------------------
# (İsteğe bağlı) libcurl DLL dosyalarını da kopyala
# ---------------------------------------------------------------
set(CURL_DLL_DIR "C:/Libraries/sfml/libcurl/bin")
file(GLOB CURL_DLLS "${CURL_DLL_DIR}/*.dll")

add_custom_command(TARGET SFML_Lidar POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CURL_DLLS}
        $<TARGET_FILE_DIR:SFML_Lidar>
        COMMENT "libcurl DLL dosyaları kopyalanıyor..."
)
