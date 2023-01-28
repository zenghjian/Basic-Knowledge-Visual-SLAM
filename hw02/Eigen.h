#include <iostream>
#include <fstream>
#include <array>
#include <vector>

#include "Eigen.h"
#include "VirtualSensor.h"


using namespace std;

struct Vertex
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	// position stored as 4 floats (4th component is supposed to be 1.0)
	Vector4f position;
	// color stored as 4 unsigned char
	Vector4uc color;
};

// true if vertices valid
bool check_valid_vertices(Vertex& v){
	if (v.position.x() != MINF) return true;
	else return false;
}
// true if edge length valid
bool check_edge_length(Vertex& v1, Vertex& v2, Vertex& v3, float edgeThreshold){

	if (!check_valid_vertices(v1) || !check_valid_vertices(v2) || !check_valid_vertices(v3)) return false;
    double edge_length_1 = (v1.position - v2.position).norm();
	double edge_length_2 = (v1.position - v3.position).norm();
	double edge_length_3 = (v2.position - v3.position).norm();
	if (edge_length_1 < edgeThreshold && edge_length_2 < edgeThreshold && edge_length_3 < edgeThreshold) return true;
	else return false;
}

bool WriteMesh(Vertex* vertices, unsigned int width, unsigned int height, const std::string& filename)
{
	float edgeThreshold = 0.01f; // 1cm

	// TODO 2: use the OFF file format to save the vertices grid (http://www.geomview.org/docs/html/OFF.html)
	// - have a look at the "off_sample.off" file to see how to store the vertices and triangles
	// - for debugging we recommend to first only write out the vertices (set the number of faces to zero)
	// - for simplicity write every vertex to file, even if it is not valid (position.x() == MINF) (note that all vertices in the off file have to be valid, thus, if a point is not valid write out a dummy point like (0,0,0))
	// - use a simple triangulation exploiting the grid structure (neighboring vertices build a triangle, two triangles per grid cell)
	// - you can use an arbitrary triangulation of the cells, but make sure that the triangles are consistently oriented
	// - only write triangles with valid vertices and an edge length smaller then edgeThreshold

	// TODO: Get number of vertices
	unsigned int nVertices = width * height;

	// TODO: Determine number of valid faces
	unsigned nFaces = 0;	

	vector<array<unsigned int, 3>> faces;
	for(auto row = 0; row < height - 1; ++row){
		for(auto col = 0; col < width - 1; ++col){
			unsigned int idx_1 = row * width + col;
			unsigned int idx_2 = (row + 1) * width + col;
			unsigned int idx_3 = row * width + col + 1; 
			unsigned int idx_4 = (row + 1) * width + col + 1;
			// first mesh
			if (check_edge_length(vertices[idx_1],vertices[idx_2],vertices[idx_3],edgeThreshold)){
				array<unsigned int, 3> face {idx_1, idx_2, idx_3};
				faces.push_back(face);
				++nFaces;
			}
			// second mesh
			if (check_edge_length(vertices[idx_2],vertices[idx_3],vertices[idx_4],edgeThreshold)){
				array<unsigned int, 3> face {idx_2, idx_3, idx_4};
				faces.push_back(face);
				++nFaces;
			}			
		}
	}
	

	// Write off file
	std::ofstream outFile(filename);
	if (!outFile.is_open()) return false;

	// write header
	outFile << "COFF" << std::endl;

	outFile << "# numVertices numFaces numEdges" << std::endl;

	outFile << nVertices << " " << nFaces << " 0" << std::endl;


	outFile << "# list of vertices" << std::endl;

	outFile << "# X Y Z R G B A" << std::endl;
	// TODO: save vertices
	for(auto idx = 0; idx < height * width ; ++idx){
		if(vertices[idx].position.x() == MINF){
			outFile << 0.0 << " " << 0.0 << " " << 0.0 << " " << 255 << " " << 255 << " " << 255 << " " << 255 << endl;
		}
		else{
			outFile << vertices[idx].position.x() 				  << " " 
					<< vertices[idx].position.y() 				  << " " 
					<< vertices[idx].position.z() 				  << " " 
					<< (unsigned int)vertices[idx].color[0] 	  << " "
					<< (unsigned int)vertices[idx].color[1] 	  << " "
					<< (unsigned int)vertices[idx].color[2] 	  << " "
					<< (unsigned int)vertices[idx].color[3] 	  
					<< endl;

					
		}
	}

	// TODO: save valid faces
	outFile << "# list of faces" << std::endl;
	outFile << "# nVerticesPerFace idx0 idx1 idx2 ..." << std::endl;

	for (auto &i : faces){
		outFile << 3 	<< " " 
				<< i[0] << " " 
				<< i[1] << " "
				<< i[2] << endl;
	}


	// close file
	outFile.close();

	return true;
}

int main()
{
	// Make sure this path points to the data folder
	std::string filenameIn = "../../Data/rgbd_dataset_freiburg1_xyz/";
	std::string filenameBaseOut = "mesh_";

	// load video
	std::cout << "Initialize virtual sensor..." << std::endl;
	VirtualSensor sensor;
	if (!sensor.Init(filenameIn))
	{
		std::cout << "Failed to initialize the sensor!\nCheck file path!" << std::endl;
		return -1;
	}

	// convert video to meshes
	while (sensor.ProcessNextFrame())
	{
		// get ptr to the current depth frame
		// depth is stored in row major (get dimensions via sensor.GetDepthImageWidth() / GetDepthImageHeight())
		float* depthMap = sensor.GetDepth();
		// get ptr to the current color frame
		// color is stored as RGBX in row major (4 byte values per pixel, get dimensions via sensor.GetColorImageWidth() / GetColorImageHeight())
		BYTE* colorMap = sensor.GetColorRGBX();

		// get depth intrinsics
		Matrix3f depthIntrinsics = sensor.GetDepthIntrinsics();
		Matrix3f depthIntrinsicsInv = depthIntrinsics.inverse();

		float fX = depthIntrinsics(0, 0);
		float fY = depthIntrinsics(1, 1);
		float cX = depthIntrinsics(0, 2);
		float cY = depthIntrinsics(1, 2);

		// compute inverse depth extrinsics
		Matrix4f depthExtrinsicsInv = sensor.GetDepthExtrinsics().inverse();

		Matrix4f trajectory = sensor.GetTrajectory();
		Matrix4f trajectoryInv = sensor.GetTrajectory().inverse();

		// TODO 1: back-projection
		// write result to the vertices array below, keep pixel ordering!
		// if the depth value at idx is invalid (MINF) write the following values to the vertices array
		// vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
		// vertices[idx].color = Vector4uc(0,0,0,0);
		// otherwise apply back-projection and transform the vertex to world space, use the corresponding color from the colormap
		Vertex* vertices = new Vertex[sensor.GetDepthImageWidth() * sensor.GetDepthImageHeight()];


		// go through all vertices
		// we assume image pixel is (x,y), 3d point is (u,v,w)

		for (int row = 0; row < sensor.GetDepthImageHeight(); ++row){
			for (int col = 0; col < sensor.GetDepthImageWidth(); ++col){
				int idx = row * sensor.GetColorImageWidth() + col;
				float w = depthMap[idx]; 
				Vector4f point_3d, point_w;		
				Vector4uc color = Vector4uc(colorMap[4*idx]  ,
											colorMap[4*idx+1],
											colorMap[4*idx+2],
											colorMap[4*idx+3]);							
				if(w == MINF){
					vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
					vertices[idx].color = Vector4uc(0,0,0,0);
				}
				else{
					float x = col;
					float y = row;
					float u = (x - cX) / fX * w;
					float v = (y - cY) / fY * w;

					point_3d = Vector4f(u, v, w, 1.0f);
					point_w = trajectoryInv * depthExtrinsicsInv * point_3d;

					vertices[idx].position = point_w;
					vertices[idx].color = color;
				}
			}
		}

		// write mesh file
		std::stringstream ss;
		ss << filenameBaseOut << sensor.GetCurrentFrameCnt() << ".off";
		if (!WriteMesh(vertices, sensor.GetDepthImageWidth(), sensor.GetDepthImageHeight(), ss.str()))
		{
			std::cout << "Failed to write mesh!\nCheck file path!" << std::endl;
			return -1;
		}

		// free mem
		delete[] vertices;
	}

	return 0;
}