#include <fc_interface.h>

Eigen::Matrix4d FC_Interface::_FK(const VectorJd& q, const Eigen::MatrixXd& DH, const Eigen::Matrix4d& base_frame, const bool& joint_rad)
{
    Eigen::Matrix3d R;
    Eigen::MatrixXd T(3, 1);
    Eigen::Matrix4d trans_mtx = Eigen::Matrix4d::Identity();
    trans_mtx = base_frame;
    Eigen::Matrix4d tmp;
    Eigen::MatrixXd DH_cur = DH;
    VectorJd q_rad = q;

    if(!joint_rad)
    {
        // Deg to Rad
        for(int i=0; i<q.rows(); i++)
        {
            q_rad(i) = q(i) * PI / 180;
        }
    }

    DH_cur.col(0) = DH.col(0) + q_rad;
    for(int i=0; i<DH_cur.rows(); i++)
    {
        R << cos(DH_cur.coeff(i, 0)), -sin(DH_cur.coeff(i, 0)) * cos(DH_cur.coeff(i, 3)),  sin(DH_cur.coeff(i, 0)) * sin(DH_cur.coeff(i, 3)),
             sin(DH_cur.coeff(i, 0)),  cos(DH_cur.coeff(i, 0)) * cos(DH_cur.coeff(i, 3)), -cos(DH_cur.coeff(i, 0)) * sin(DH_cur.coeff(i, 3)),
             0,                        sin(DH_cur.coeff(i, 3)),                            cos(DH_cur.coeff(i, 3));

        T << DH_cur.coeff(i, 2) * cos(DH_cur.coeff(i, 0)), 
             DH_cur.coeff(i, 2) * sin(DH_cur.coeff(i, 0)), 
             DH_cur.coeff(i, 1);
        tmp << R, T, 0, 0, 0, 1;
        trans_mtx = trans_mtx * tmp;
    }
    return trans_mtx;
}

Eigen::MatrixXd FC_Interface::loadFromFile_(std::string file_path, std::string key)
{
	Eigen::MatrixXd data;

	// READ JSON FILE
	std::ifstream file(file_path);
	Json::Value root;
	file >> root;

	// PARSE JSON FILE
	Json::Value data_json = root[key];

	// CONVERT TO EIGEN MATRIX
	data.resize(data_json.size(), data_json[0].size());
	for(int i=0; i<data_json.size(); i++)
	{
		for(int j=0; j<data_json[0].size(); j++)
			data(i, j) = data_json[i][j].asDouble();
	}

	// CLOSE FILE
	file.close();

	return data;
}