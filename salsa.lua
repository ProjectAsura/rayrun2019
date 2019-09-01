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
		"thirdparty/imgui/examples/imgui_impl_win32.cpp",
		"thirdparty/imgui/examples/imgui_impl_win32.h",
		"thirdparty/imgui/examples/imgui_impl_dx11.cpp",
		"thirdparty/imgui/imgui.cpp",
		"thirdparty/imgui/imgui_demo.cpp",
		"thirdparty/imgui/imgui_draw.cpp",
		"thirdparty/imgui/imgui_widgets.cpp",
	}
	buildoptions { 
	"/openmp"
	}
	includedirs {
		"thirdparty/stb/",
		"thirdparty/tinyobjloader/",
		"thirdparty/picojson/",
		"thirdparty/imgui/",
		"thirdparty/imgui/examples",
		"thirdparty/glm/",
	}
	links {
		"D3D11.lib"
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
		"salsa/src/s3d_bvh.cpp",
		"src/rayrun.hpp",
	}
	includedirs {
		"salsa/include/",
		"src/",
	}	
	cppdialect "C++17"