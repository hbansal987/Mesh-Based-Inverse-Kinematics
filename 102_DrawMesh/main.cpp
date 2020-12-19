#include "tutorial_shared_path.h"
#include <igl/opengl/glfw/Viewer.h>
#include <GLFW/glfw3.h>
#include <string>
#include <iostream>
#include <map>
#include <igl/readOBJ.h>
#include <igl/readOFF.h>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <unsupported/Eigen/MatrixFunctions>

using namespace Eigen;

Eigen::MatrixXd V;
Eigen::MatrixXi F;

Eigen::MatrixXd V2;
Eigen::MatrixXi F2;

Eigen::MatrixXd V3;
Eigen::MatrixXi F3;

Eigen::MatrixXd V2_inverse;

int main(int argc, char * argv[])
{

////////////////////////////////// MILESTONE 1 //////////////////////////////////////////////	

  igl::opengl::glfw::Viewer viewer;
  const auto names =
    {"object2.obj","object3.obj","object.obj"};
  std::map<int, Eigen::RowVector3d> colors;
  int last_selected = -1;
  for(const auto & name : names)
  {
    viewer.load_mesh_from_file(std::string(TUTORIAL_SHARED_PATH) + "/" + name);
    colors.emplace(viewer.data().id, 0.5*Eigen::RowVector3d::Random().array() + 0.5);
  }

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


  igl::readOBJ(TUTORIAL_SHARED_PATH "/object.obj", V, F); //Original
  igl::readOBJ(TUTORIAL_SHARED_PATH "/object2.obj", V2, F2); //Reference
  igl::readOBJ(TUTORIAL_SHARED_PATH "/object3.obj", V3, F3); //Reference 2

  //std::cout<<V<<std::endl;
  //std::cout<<V.size()<<std::endl;
  //std::cout<<F<<std::endl;
  //std::cout<<F.size()<<std::endl;


  // Formation of feature vector 
  // f = Gx

  // F has the indices of the vertices used to form the triangle


  ///////////////////////////////////// MILESTONE 2 ////////////////////////////////////////

  int number_of_triangles = F.size()/3; 
  int number_of_vertices = V.size(); 
  //int number_of_vertices = 3;

  //This is for the original matrix not the reference. All the vertices are put in a single matrix form (x1...xn,y1....yn,z1.....zn)
  Eigen::MatrixXd x_matrix(3*number_of_vertices,1);
  for(int i=0;i<number_of_vertices/3;i=i+3){
      //std::cout<<V(i,0)<<std::endl;
      x_matrix(i,0) = V(i,0);
      //std::cout<<V(i,0)<<std::endl;
      x_matrix(i+1,0) = V(i,1);
      x_matrix(i+2,0) = V(i,2);
  }

  //std::cout<<F.size()<<std::endl;

  //This is for the reference matrix.
  Eigen::MatrixXd G_matrix_small(3*number_of_triangles,number_of_vertices);
  Eigen::MatrixXd G_matrix_small2(3*number_of_triangles,number_of_vertices);
  for(int i=0;i<3*number_of_triangles;i++){
    for(int j=0;j<number_of_vertices;j++){
      G_matrix_small(i,j) = 0;
      G_matrix_small2(i,j) = 0;   
    }
  }

  //////////////////////  REFERENCE ONE /////////////////////

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

  //std::cout<<F.size()<<std::endl;

  Eigen::MatrixXd G_matrix(9*number_of_triangles,3*number_of_vertices);
  for(int i=0;i<9*number_of_triangles;i++){
   for(int j=0;j<3*number_of_vertices;j++){
     G_matrix(i,j) = 0;
   }
 }

  //std::cout<<"ONE"<<std::endl;
  for(int i=0;i<3*number_of_triangles;i++){
    for(int j=0;j<number_of_vertices;j++){
      G_matrix(i,j) = G_matrix_small(i,j);
    }
  }

  for(int i=3*number_of_triangles;i<6*number_of_triangles;i++){
    for(int j=number_of_vertices;j<2*number_of_vertices;j++){
      G_matrix(i,j) = G_matrix_small(i-3*number_of_triangles,j-number_of_vertices);
    }
  }

  for(int i=6*number_of_triangles;i<9*number_of_triangles;i++){
    for(int j=2*number_of_vertices;j<3*number_of_vertices;j++){
      G_matrix(i,j) = G_matrix_small(i-6*number_of_triangles,j-2*number_of_vertices);
    }
  }

  ///////////////////////// REFERENCE TWO ///////////////////////////////
  for(int i=0;i<number_of_triangles;i++){
  
  int vertex1 = F3(i,0);
  int vertex2 = F3(i,1);
  int vertex3 = F3(i,2);

  G_matrix_small2(3*i,vertex1) = V3(vertex1,0);
  G_matrix_small2(3*i+1,vertex1) = V3(vertex1,1);
  G_matrix_small2(3*i+2,vertex1) = V3(vertex1,2);

  G_matrix_small2(3*i,vertex2) = V3(vertex2,0);
  G_matrix_small2(3*i+1,vertex2) = V3(vertex2,1);
  G_matrix_small2(3*i+2,vertex2) = V3(vertex2,2);

  G_matrix_small2(3*i,vertex3) = V3(vertex3,0);
  G_matrix_small2(3*i+1,vertex3) = V3(vertex3,1);
  G_matrix_small2(3*i+2,vertex3) = V3(vertex3,2);

  }

  //std::cout<<F.size()<<std::endl;

  Eigen::MatrixXd G_matrix2(9*number_of_triangles,3*number_of_vertices);
  for(int i=0;i<9*number_of_triangles;i++){
   for(int j=0;j<3*number_of_vertices;j++){
     G_matrix2(i,j) = 0;
   }
 }

  //std::cout<<"ONE"<<std::endl;
  for(int i=0;i<3*number_of_triangles;i++){
    for(int j=0;j<number_of_vertices;j++){
      G_matrix2(i,j) = G_matrix_small2(i,j);
    }
  }

  for(int i=3*number_of_triangles;i<6*number_of_triangles;i++){
    for(int j=number_of_vertices;j<2*number_of_vertices;j++){
      G_matrix2(i,j) = G_matrix_small2(i-3*number_of_triangles,j-number_of_vertices);
    }
  }

  for(int i=6*number_of_triangles;i<9*number_of_triangles;i++){
    for(int j=2*number_of_vertices;j<3*number_of_vertices;j++){
      G_matrix2(i,j) = G_matrix_small2(i-6*number_of_triangles,j-2*number_of_vertices);
    }
  }


  Eigen::MatrixXd f1 = G_matrix*x_matrix;
  Eigen::MatrixXd f2 = G_matrix2*x_matrix;
  //std::cout<<f2<<std::endl;


  ///////////////////////////////////////// MILESTONE 3 /////////////////////////////////

  //l=2 in this case
  Eigen::MatrixXd M_matrix(9*number_of_triangles,2);
  for(int i=0;i<9*number_of_triangles;i++){
  		M_matrix(i,0) = f1(i,0);
  		M_matrix(i,1) = f2(i,0);
  }
  //std::cout<<"F1 "<<f1.size()<<std::endl;
  //std::cout<<"F2 "<<f2.size()<<std::endl;
  //std::cout<<M_matrix<<std::endl;


  
  //Taking c = Gx for the first arbitray value of x
  Eigen::MatrixXd c = G_matrix*x_matrix;

  Eigen::MatrixXd A(2+3*number_of_vertices,2+3*number_of_vertices);
  A.block(2,2,0,0)=M_matrix.transpose()*M_matrix;
  A.block(2,3*number_of_vertices,0,2) = -M_matrix.transpose()*G_matrix;
  A.block(3*number_of_vertices,2,2,0) = -G_matrix.transpose()*M_matrix;
  A.block(3*number_of_vertices,3*number_of_vertices,2,2) = G_matrix.transpose()*G_matrix;

  Eigen::MatrixXd b(2+3*number_of_vertices,1);
  b.block(2,1,0,0) = -M_matrix.transpose()*c;
  b.block(3*number_of_vertices,1,2,0) = G_matrix.transpose()*c;

  Eigen::MatrixXd x_ans  = (((A.transpose()*A).inverse())*A.transpose())*b;
  Eigen::MatrixXd w_matrix = x_ans.block<2,1>(0,0);

  //std::cout<<w_matrix<<std::endl;

  //**************** not a good value for w right now ******************

  //////////////////////////////////////// MILESTONE 4 /////////////////////////////////

  //A=UΣV* --> This is SVD decomposition  
  //M=(UΣU∗)(UV∗)=(UV∗)(VΣV∗) --> This is UV decomposition using SVD decomposition

  Eigen::MatrixXd R1j;
  Eigen::MatrixXd S1j;

  Eigen::MatrixXd R2j;
  Eigen::MatrixXd S2j;

  Eigen::MatrixXd T1j; 
  Eigen::MatrixXd T2j;

  Eigen::MatrixXd Tj;
  Eigen::MatrixXd Jacobian_Tj; 

  Eigen::MatrixXd Mw(9*number_of_triangles,1);
  Eigen::MatrixXd Mw_jacobian(9*number_of_triangles,1);
  int wk = 1;

  int count=0;
  
  for(int i=0;i<number_of_triangles/3;i++){

  	int vertex11 = F(i,0);
  	int vertex12 = F(i,1);
  	int vertex13 = F(i,2);

  	int vertex21 = F2(i,0);
  	int vertex22 = F2(i,1);
  	int vertex23 = F2(i,2);

  	int vertex31 = F3(i,0);
  	int vertex32 = F3(i,1);
  	int vertex33 = F3(i,2);

  	Eigen::MatrixXd V_matrix1(3,3);
  	Eigen::MatrixXd V_matrix2(3,3);
  	Eigen::MatrixXd V_matrix3(3,3);

  	V_matrix1(0,0) = V(vertex11,0);
  	V_matrix1(0,1) = V(vertex11,1);
  	V_matrix1(0,2) = V(vertex11,2);
  	V_matrix1(1,0) = V(vertex12,0);
  	V_matrix1(1,1) = V(vertex12,1);
  	V_matrix1(1,2) = V(vertex12,2);
  	V_matrix1(2,0) = V(vertex13,0);
  	V_matrix1(2,1) = V(vertex13,1);
  	V_matrix1(2,2) = V(vertex13,2);


  	V_matrix2(0,0) = V2(vertex21,0);
  	V_matrix2(0,1) = V2(vertex21,1);
  	V_matrix2(0,2) = V2(vertex21,2);
  	V_matrix2(1,0) = V2(vertex22,0);
  	V_matrix2(1,1) = V2(vertex22,1);
  	V_matrix2(1,2) = V2(vertex22,2);
  	V_matrix2(2,0) = V2(vertex23,0);
  	V_matrix2(2,1) = V2(vertex23,1);
  	V_matrix2(2,2) = V2(vertex23,2);


  	V_matrix3(0,0) = V3(vertex31,0);
  	V_matrix3(0,1) = V3(vertex31,1);
  	V_matrix3(0,2) = V3(vertex31,2);
  	V_matrix3(1,0) = V3(vertex32,0);
  	V_matrix3(1,1) = V3(vertex32,1);
  	V_matrix3(1,2) = V3(vertex32,2);
  	V_matrix3(2,0) = V3(vertex33,0);
  	V_matrix3(2,1) = V3(vertex33,1);
  	V_matrix3(2,2) = V3(vertex33,2);

  	T1j = V_matrix2*V_matrix1;
  	T2j = V_matrix3*V_matrix1;

  	JacobiSVD<MatrixXd> svd1( T1j, ComputeThinU | ComputeThinV);
  	JacobiSVD<MatrixXd> svd2( T2j, ComputeThinU | ComputeThinV);

  	//A = WSV* (SVD decomposition)
  	//P = VSV*
  	//U = WV*

  	R1j = svd1.matrixU()*svd1.matrixV().transpose();
  	S1j = svd1.matrixV()*svd1.singularValues().asDiagonal()*svd1.matrixV().transpose();

  	R2j = svd2.matrixU()*svd2.matrixV().transpose();
  	S2j = svd2.matrixV()*svd2.singularValues().asDiagonal()*svd2.matrixV().transpose();

  	//Eigen::MatrixXd wi();
  	
  	Tj = ((wk*R1j.log()+wk*R2j.log()).exp()) *(wk*S1j+wk*S2j);

  	Jacobian_Tj = (R1j.log()+R2j.log()).exp()*R1j.log()*(S1j+S2j) + (R1j.log()+R2j.log()).exp()*S1j;

  	for(int r=0;r<3;r++){
  		Mw(count,0) = Tj(r,0);
  		Mw(count+1,0) = Tj(r,1);
  		Mw(count+2,0) = Tj(r,2);
  		count = count+3;
  	}

  	for(int r=0;r<3;r++){
  		Mw_jacobian(count,0) = Jacobian_Tj(r,0);
  		Mw_jacobian(count+1,0) = Jacobian_Tj(r,1);
  		Mw_jacobian(count+2,0) = Jacobian_Tj(r,2);
  		count = count+3;
  	}
  
  }



  //std::cout<<T1j<<std::endl;
  //std::cout<<"Hello"<<std::endl;
  //std::cout<<R1j*S1j<<std::endl;
  //std::cout<<Tj<<std::endl;
  //std::cout<<Mw<<std::endl;

  /////////////// Gauss Newton Algorithm //////////////////////

  //Stopping conditions:

  int deltaK = 1;
  wk = wk + deltaK;

  Eigen::MatrixXd A_matrix(9*number_of_triangles,3*number_of_vertices);
  A_matrix.block(3*number_of_triangles,number_of_vertices,0,0) = G_matrix;
  A_matrix.block(3*number_of_triangles,number_of_vertices,3*number_of_triangles,number_of_vertices) = G_matrix;
  A_matrix.block(3*number_of_triangles,number_of_vertices,2*3*number_of_triangles,2*number_of_vertices) = G_matrix;

  A_matrix.block(9*number_of_triangles,1,3*number_of_vertices,0) = Mw_jacobian;

  //std::cout<<A<<std::endl;



  viewer.launch();
  return EXIT_SUCCESS;
}
