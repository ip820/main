#include <opencv2\opencv.hpp>
#include <armadillo.hpp>

#include <vector>
#include <string>
#include <math.h>

#include "Definition.h"
#include "Epipolar.h"

using namespace std;
using namespace arma;



arma::mat33 Epipolar::computeRotation
	(/* input */ double om, double ph, double kp, ROT_ORDER flag)
{
	arma::mat33 Rx, Ry, Rz, R;

	Rx
		<< 1 << 0 << 0 << endr
		<< 0 << cos(om) << -sin(om) << endr
		<< 0 << sin(om) << cos(om) << endr;

	Ry 
		<< cos(ph) << 0 << sin(ph) << endr
		<< 0 << 1 << 0 << endr
		<< -sin(ph) << 0 << cos(ph) << endr;

	Rz 
		<< cos(kp) << -sin(kp) << 0 <<endr
		<< sin(kp) << cos(kp) << 0 << endr
		<< 0 << 0 << 1 << endr;

	if(flag == XYZ)
	{
		R = arma::mat33(Rx * Ry * Rz);
	}
	else if(flag == ZYX)
	{
		R = arma::mat33(Rz * Ry * Rx);
	}
	else
	{
		cout << "ERROR : Check your flag for rotation order!!" << endl;
		R.zeros();
	}

	// DEBUG
	// cout << "R" << endl << R << endl;
	return R;
}

arma::mat::fixed<4, 1> Epipolar::computeImgPlane(EO& eo, IO& io)
{
	arma::mat::fixed<4, 1> coeff_ip;   // coefficients of image plane equation
	arma::mat::fixed<3, 1> n_mc,       // normal vector of image plane in MCS
						   n_cz,       // normal vector of image plane in CCS
						   N_mc;       // Normalized normal vector of image plane in MCS
	arma::mat::fixed<3, 3> r_mc, r_cm; // rotation matrix which rotate MCS to CCS
	double D;                          // coefficient of plane equation, Ax+By+Cz+D=0

	arma::mat::fixed<3, 1> pos;

	pos
		<< eo.Xc << endr
		<< eo.Yc << endr
		<< eo.Zc << endr;

	n_cz
		<< 0 << endr
		<< 0 << endr
		<< 1 << endr;

	r_mc = Epipolar::computeRotation(eo.omega, eo.phi, eo.kappa, ZYX);

	r_cm = arma::mat::fixed<3, 3>(arma::trans(r_mc));

	// DEBUG
	// cout << "Rotation matrix MCS 2 CCS" << endl << r_mc << endl;
	// cout << "Rotation matrix CCS 2 MCS" << endl << r_cm << endl;

	n_mc = arma::mat::fixed<3, 1>(r_cm * n_cz);

	// DEBUG
	// cout << "image plane normal vector" << endl << n_mc << endl;

	N_mc
		<< n_mc.at(0, 0) / n_mc.at(2, 0) << endr
		<< n_mc.at(1, 0) / n_mc.at(2, 0) << endr
		<< n_mc.at(2, 0) / n_mc.at(2, 0) << endr;

	// DEBUG
	// cout << "normalized img plane normal vector" << endl << N_mc << endl;

	D = io.f_ccs * norm(N_mc) - dot(N_mc, pos);

	//cout << io.f_ccs << endl;

	coeff_ip
		<< N_mc.at(0, 0) << endr
		<< N_mc.at(1, 0) << endr
		<< N_mc.at(2, 0) << endr
		<< D << endr;

	// DEBUG
	// cout << "coefficients of image Plane equation" << endl << coeff_ip << endl;

	return coeff_ip;
}

arma::mat::fixed<3, 1> Epipolar::convertIP2MCS(EO& eo, IO& io, cv::Point2f& ip)
{
	arma::mat::fixed<3, 1> ip_ccs, ip_mcs;
	arma::mat::fixed<3, 1> C_mcs;
	arma::mat::fixed<3, 3> R_mc, R_cm;

	C_mcs
		<< eo.Xc << endr
		<< eo.Yc << endr
		<< eo.Zc << endr;

	R_mc = Epipolar::computeRotation(eo.omega, eo.phi, eo.kappa, ZYX);

	R_cm = arma::mat::fixed<3, 3>(arma::trans(R_mc));

	ip_ccs 
		<< (ip.x - io.pp_pcs.x) * io.size_pixel.width << endr
		<< (-ip.y + io.pp_pcs.y) * io.size_pixel.width << endr
		<< -(io.f_ccs) << endr;

	ip_mcs = arma::mat::fixed<3, 1>((R_cm * ip_ccs) + C_mcs);

	//cout << "ip_ccs" << endl;
	//cout << ip_ccs << endl;

	//cout << "C_mcs" << endl;
	//cout << C_mcs << endl;

	//cout << "ip_mcs" << endl;
	//cout << ip_mcs << endl;

	return ip_mcs;
}

void Epipolar::computeEpipole
	(/* output */ 
	arma::mat::fixed<3, 1>& epp1, arma::mat::fixed<3, 1>& epp2,
	arma::mat::fixed<4, 1>& img1, arma::mat::fixed<4, 1>& img2)
{
	arma::mat::fixed<3, 1> d_C1C2;  // direction vector of Perspective Center Line
	arma::mat::fixed<3, 1> C1,      // position of perspective center of EO1
						   C2;      // position of perspective center of EO2

	double t1, t2;

	
	C1
		<< Epipolar::eo1.Xc << endr
		<< Epipolar::eo1.Yc << endr
		<< Epipolar::eo1.Zc << endr;
	C2
		<< Epipolar::eo2.Xc << endr
		<< Epipolar::eo2.Yc << endr
		<< Epipolar::eo2.Zc << endr;

	d_C1C2 = arma::mat::fixed<3, 1>(C1 - C2);

	// DEBUG
	// cout << "Perspective Center Line" << endl <<d_C1C2 << endl;

	img1 = Epipolar::computeImgPlane(Epipolar::eo1, Epipolar::io);
	img2 = Epipolar::computeImgPlane(Epipolar::eo2, Epipolar::io);

	// DEBUG
	// cout << "image plane 1 in MCS" << endl << img1 << endl;
	// cout << "image plane 2 in MCS" << endl << img2 << endl;


	t1 = - (arma::dot(img1.submat(0, 0, 2, 0), C1) + img1(3, 0)) 
		/ arma::dot(img1.submat(0, 0, 2, 0), d_C1C2);
	
	t2 = - (arma::dot(img2.submat(0, 0, 2, 0), C2) + img2(3, 0)) 
		/ arma::dot(img2.submat(0, 0, 2, 0), d_C1C2);


	epp1 = arma::mat::fixed<3, 1>(C1 + (t1 * d_C1C2));
	epp2 = arma::mat::fixed<3, 1>(C2 + (t2 * d_C1C2));

}

arma::mat::fixed<4, 1> Epipolar::determineEpipolarPlane
	(/* input */ EO& eo, IO& io, arma::mat::fixed<3, 1>& epp, cv::Point2f& ip)
{
	arma::mat::fixed<3, 1> ip_mcs;
	arma::mat::fixed<3, 1> C_mcs;

	arma::mat::fixed<3, 1> epp_vec, ip_vec;
	arma::mat::fixed<3, 1> n_epi, N_epi; 
	double D_epi;

	arma::mat::fixed<4, 1> result_epi;

	C_mcs
		<< eo.Xc << endr
		<< eo.Yc << endr
		<< eo.Zc << endr;

	ip_mcs = Epipolar::convertIP2MCS(eo, io, ip);

	// DEBUG
	// cout << ip_mcs << endl;

	epp_vec = arma::mat::fixed<3, 1>(epp - C_mcs);
	ip_vec = arma::mat::fixed<3, 1>(ip_mcs - C_mcs);

	n_epi = arma::mat::fixed<3, 1>(arma::cross(epp_vec, ip_vec));

	N_epi 
		<< n_epi(0, 0) / n_epi(2, 0) << endr
		<< n_epi(1, 0) / n_epi(2, 0) << endr
		<< n_epi(2, 0) / n_epi(2, 0) << endr;

	// DEBUG
	//cout << "normal vector of epipolar plane" << endl << N_epi << endl;

	D_epi = -arma::dot(N_epi, ip_mcs);

	// DEBUG
	//cout << D_epi << endl;

	result_epi
		<< N_epi(0, 0) << endr
		<< N_epi(1, 0) << endr
		<< N_epi(2, 0) << endr
		<< D_epi << endr;

	return result_epi;
}

double Epipolar::computeDistance
	(/* input */ 
		EO& eo, IO& io,
		arma::mat::fixed<4, 1>& img_plane, 
		arma::mat::fixed<4, 1>& epi_plane,
		arma::mat::fixed<3, 1>& epipole,
		arma::mat::fixed<3, 1>& img_point,
		/* output */Vec3f& el_dir)
{
	arma::mat::fixed<3, 1> inter_dir;
	arma::mat::fixed<3, 1> inter_point;
	
	arma::mat::fixed<3, 1> tmp_point;
	double t_tmp;

	cv::Point2f epp_ccs, tmp_ccs, ip_ccs;

	double distance;

	inter_dir = arma::mat::fixed<3, 1>(arma::cross(img_plane.submat(0, 0, 2, 0), epi_plane.submat(0, 0, 2, 0)));

	inter_point
		<< epipole.at(0, 0) << endr
		<< epipole.at(1, 0) << endr
		<< epipole.at(2, 0) << endr;

	// DEBUG
	//cout << "inter_dir" << endl << inter_dir << endl;
	//cout << "inter_pt" << endl << inter_point << endl;

	t_tmp = 10;

	tmp_point = arma::mat::fixed<3, 1>(t_tmp * inter_dir + inter_point);

	epp_ccs = Epipolar::convertIP2CCS(eo, io, epipole);
	tmp_ccs = Epipolar::convertIP2CCS(eo, io, tmp_point);
	ip_ccs = Epipolar::convertIP2CCS(eo, io, img_point);

	double a = (tmp_ccs.y - epp_ccs.y);
	double b = -(tmp_ccs.x - epp_ccs.x);
	double c = (-a * epp_ccs.x) + (-b * epp_ccs.y);

	el_dir[0] = a * io.size_pixel.width;
	el_dir[1] = -b * io.size_pixel.width;
	el_dir[2] = c - (a * io.size_pixel.width * io.pp_pcs.x) + (b * io.size_pixel.width * io.pp_pcs.y);

	// DEBUG
	// cout << "el_dir" << el_dir << endl;

	distance = ((a * ip_ccs.x) + (b * ip_ccs.y) + c) / sqrt((a * a) + (b * b));

	if (distance < 0)
		return -distance;
	else
		return distance;
}

cv::Point2f Epipolar::convertIP2CCS
	(/* input */
	EO& eo, IO& io, arma::mat::fixed<3, 1>& ip_mcs)
{
	arma::mat::fixed<3, 3> R;

	arma::mat::fixed<3, 1> tmp, C;
	
	cv::Point2f ip;
	double scale;

	C 
		<< eo.Xc << endr
		<< eo.Yc << endr
		<< eo.Zc << endr;

	R = Epipolar::computeRotation(eo.omega, eo.phi, eo.kappa, ZYX);

	tmp = arma::mat::fixed<3, 1> (R * (ip_mcs - C));

	scale = io.f_ccs / tmp.at(2, 0);

	ip.x = tmp.at(0, 0) * scale;
	ip.y = tmp.at(1, 0) * scale;

	return ip;
}

bool Epipolar::checkCrossEpipolar
	(/* input */ cv::Point2f ip1, cv::Point2f ip2)
{

	arma::mat::fixed<4, 1> eplane1, eplane2;
	arma::mat::fixed<3, 1> ip_mcs1, ip_mcs2;
	Vec3f eline1, eline2;

	double dist1, dist2;
	double px_dist1, px_dist2;
	
	// DEBUG
	//cout << "img1 : " << endl
	//	<< img_plane1 << endl;

	//cout << "img2 : " << endl
	//	<< img_plane2 << endl;

	//cout << "epp1" << endl;
	//cout << epipole1 << endl;

	//cout << "epp2" << endl;
	//cout << epipole2 << endl;


	eplane1 = Epipolar::determineEpipolarPlane(Epipolar::eo1, Epipolar::io, Epipolar::epipole1, ip1);
	eplane2 = Epipolar::determineEpipolarPlane(Epipolar::eo2, Epipolar::io, Epipolar::epipole2, ip2);

	// DEBUG
	//cout << "Epipolar plane 1" << endl << eplane1 << endl;
	//cout << "Epipolar plane 2" << endl << eplane2 << endl;

	ip_mcs1 = Epipolar::convertIP2MCS(Epipolar::eo1, Epipolar::io, ip1);
	ip_mcs2 = Epipolar::convertIP2MCS(Epipolar::eo2, Epipolar::io, ip2);

	// DEBUG
	//cout << "ip_mcs1" << endl << ip_mcs1 << endl;
	//cout << "ip_mcs2" << endl << ip_mcs2 << endl;


	dist1 = Epipolar::computeDistance(Epipolar::eo2, Epipolar::io, 
		Epipolar::img_plane2, eplane1, Epipolar::epipole2, ip_mcs2, eline2);
	dist2 = Epipolar::computeDistance(Epipolar::eo1, Epipolar::io, 
		Epipolar::img_plane1, eplane2, Epipolar::epipole1, ip_mcs1, eline1);

	px_dist1 = dist1 / Epipolar::io.size_pixel.width;
	px_dist2 = dist2 / Epipolar::io.size_pixel.width;

	// DEBUG
	//cout << "epi distance in pixel" << endl << px_dist1 << endl;
	//cout << "epi distance in pixel" << endl << px_dist2 << endl;

	if(px_dist1 < Epipolar::threshold 
		&& px_dist2 < Epipolar::threshold)
	{
		Epipolar::elines1.push_back(eline1);
		Epipolar::elines2.push_back(eline2);
		return 1;
	}
	else
		return 0;
}


Epipolar::Epipolar(EO& eo_1, EO& eo_2, IO& io_, double th)
{
	Epipolar::eo1 = eo_1;
	Epipolar::eo2 = eo_2;
	Epipolar::io = io_;
	Epipolar::threshold = th;

	Epipolar::computeEpipole
		(/* output */ Epipolar::epipole1, Epipolar::epipole2, Epipolar::img_plane1, Epipolar::img_plane2);

	cout << "success" << endl;
}
