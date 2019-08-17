-- solution 
solution "my_rayrun"
	location "generated"
	configurations { "Debug", "Release" }
	platforms {"x64"}
	--
	configuration "Debug"
		defines { "DEBUG" }
		symbols "On"
	--
	configuration "Release"
		defines { "NDEBUG", "NO_ASSERT" }
		optimize "On"

-- 
project "my_rayrun"
	kind "ConsoleApp"
	language "C++"
	characterset "MBCS"
	files {
		"src/main.cpp",
		"src/rayrun.hpp",
	}
	includedirs {
		"thirdparty/stb/",
		"thirdparty/tinyobjloader/",
		"thirdparty/picojson/",
	}
	cppdialect "C++17"
	dependson { "salsa" }

-- 
project "salsa"
	kind "SharedLib"
	language "C++"
	characterset "MBCS"
	files {
		"salsa/include/s3d_math.h",
		"salsa/include/s3d_bvh.h",
		"salsa/src/dll_main.cpp",
		"src/rayrun.hpp",
	}
	includedirs {
		"salsa/include/",
		"src/",
	}	
	cppdialect "C++17"