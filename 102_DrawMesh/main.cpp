#include "tutorial_shared_path.h"
#include <igl/opengl/glfw/Viewer.h>
#include <GLFW/glfw3.h>
#include <string>
#include <iostream>
#include <map>
#include <igl/readOBJ.h>

Eigen::MatrixXd V;
Eigen::MatrixXi F;

Eigen::MatrixXd V2;
Eigen::MatrixXi F2;

Eigen::MatrixXd V2_inverse;

int main(int argc, char * argv[])
{
  igl::opengl::glfw::Viewer viewer;
  const auto names =
    {"lion-01.obj","lion-02.obj","lion-03.obj","lion-04.obj","lion-05.obj"};
  std::map<int, Eigen::RowVector3d> colors;
  int last_selected = -1;
  for(const auto & name : names)
  {
    viewer.load_mesh_from_file(std::string(TUTORIAL_SHARED_PATH) + "/" + name);
    colors.emplace(viewer.data().id, 0.5*Eigen::RowVector3d::Random().array() + 0.5);
  }

  viewer.callback_key_down =
    [&](igl::opengl::glfw::Viewer &, unsigned int key, int mod)
  {
    if(key == GLFW_KEY_BACKSPACE)
    {
      int old_id = viewer.data().id;
      if (viewer.erase_mesh(viewer.selected_data_index))
      {
        colors.erase(old_id);
        last_selected = -1;
      }
      return true;
    }
    return false;
  };

  // Refresh selected mesh colors
  viewer.callback_pre_draw =
    [&](igl::opengl::glfw::Viewer &)
  {
    if (last_selected != viewer.selected_data_index)
    {
      for (auto &data : viewer.data_list)
      {
        data.set_colors(colors[data.id]);
      }
      viewer.data_list[viewer.selected_data_index].set_colors(Eigen::RowVector3d(0.9,0.1,0.1));
      last_selected = viewer.selected_data_index;
    }
    return false;
  };


  igl::readOBJ(TUTORIAL_SHARED_PATH "/lion-01.obj", V, F); //Original
  igl::readOBJ(TUTORIAL_SHARED_PATH "/lion-02.obj", V2, F2); //Reference

  std::cout<<F.size()<<std::endl;
  std::cout<<V.size()<<std::endl;    

  //Formation of feature vector 
  // f = Gx

  //F has the indices of the vertices used to form the triangle

  int number_of_triangles = F.size()/3; 
  int number_of_vertices = V.size(); 

  //This is for the original matrix not the reference. All the vertices are put in a single matrix form (x1...xn,y1....yn,z1.....zn)
  
  Eigen::MatrixXd x_matrix(3*number_of_vertices,1);
  for(int i=0;i<number_of_vertices/3;i=i+3){
      x_matrix(i,0) = V(i,0);
      x_matrix(i+1,0) = V(i,1);
      x_matrix(i+2,0) = V(i,2);
  }

  //This is for the reference matrix.
  Eigen::MatrixXd G_matrix_small(3*number_of_triangles,number_of_vertices);
  for(int i=0;i<3*number_of_triangles;i++){
    for(int j=0;j<number_of_vertices;j++){
      G_matrix_small(i,j) = 0;   
    }
  }

  for(int i=0;i<number_of_triangles;i++){
  
  int vertex1 = F2(i,0);
  int vertex2 = F2(i,1);
  int vertex3 = F2(i,2);

  G_matrix_small(3*i,vertex1) = V2(vertex1,0);
  G_matrix_small(3*i+1,vertex1) = V2(vertex1,1);
  G_matrix_small(3*i+2,vertex1) = V2(vertex1,2);

  G_matrix_small(3*i,vertex2) = V2(vertex2,0);
  G_matrix_small(3*i+1,vertex2) = V2(vertex2,1);
  G_matrix_small(3*i+2,vertex2) = V2(vertex2,2);

  G_matrix_small(3*i,vertex3) = V2(vertex3,0);
  G_matrix_small(3*i+1,vertex3) = V2(vertex3,1);
  G_matrix_small(3*i+2,vertex3) = V2(vertex3,2);

  }


  Eigen::MatrixXd G_matrix(9*number_of_triangles,3*number_of_vertices);
  for(int i=0;i<9*number_of_triangles;i++){
    for(int j=0;i<3*number_of_vertices;j++){
      G_matrix(i,j) = 0;
    }
  }

  for(int i=0;i<3*number_of_triangles;i++){
    for(int j=0;i<number_of_vertices;j++){
      G_matrix(i,j) = G_matrix_small(i,j);
    }
  }

  for(int i=3*number_of_triangles;i<6*number_of_triangles;i++){
    for(int j=number_of_vertices;i<2*number_of_vertices;j++){
      G_matrix(i,j) = G_matrix_small(i-3*number_of_triangles,j-number_of_vertices);
    }
  }

  for(int i=6*number_of_triangles;i<9*number_of_triangles;i++){
    for(int j=2*number_of_vertices;i<3*number_of_vertices;j++){
      G_matrix(i,j) = G_matrix_small(i-6*number_of_triangles,j-2*number_of_vertices);
    }
  }

  Eigen::MatrixXd f = G_matrix*x_matrix;

  viewer.launch();
  return EXIT_SUCCESS;
}
