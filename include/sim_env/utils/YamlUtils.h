//
// Created by joshua on 4/25/17.
//

#ifndef SIM_ENV_YAMLUTILS_H
#define SIM_ENV_YAMLUTILS_H

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include "sim_env/SimEnv.h"

namespace YAML {

//    template< typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows = _Rows, int _MaxCols = _Cols >
    template< typename _Scalar, int _Rows, int _Cols>
    struct convert< Eigen::Matrix< _Scalar, _Rows, _Cols> > {

        static Node encode(const Eigen::Matrix< _Scalar, _Rows, _Cols>& matrix) {
            Node node;
            long num_elements = matrix.cols() * matrix.rows();
            for (int i = 0; i < num_elements; ++i) {
                node.push_back(matrix(i / matrix.cols(), i % matrix.cols()));
            }
            return node;
        }

        static bool decode(const Node &node, Eigen::Matrix< _Scalar, _Rows, _Cols> &matrix) {
            size_t num_elements = node.size();
            sim_env::LoggerPtr logger = sim_env::DefaultLogger::getInstance();
            // The desired dimension is governed by _Rows and _Cols
            if (_Rows == Eigen::Dynamic && _Cols == Eigen::Dynamic) {
                // There is no way to decide what format the matrix should be, so we fail here
                logger->logErr("Could not decode Eigen::Matrix. _Rows and _Cols can not be dynamic at the same time.",
                                "sim_env/YamlUtils.h");
                return false;
            }

            // Check if we have dynamic rows and specified number of columns
            if (_Rows == Eigen::Dynamic) {
                if (num_elements % _Cols != 0) {
                    logger->logErr("Could not decode Eigen::Matrix. Number of elements is not a multiple"
                                   " of number of requested columns.", "sim_env/YamlUtils.h");
                    return false;
                }
                matrix.resize(num_elements / _Cols, Eigen::NoChange);
            } else if (_Cols == Eigen::Dynamic) {
                if (num_elements % _Rows != 0) {
                    logger->logErr("Could not decode Eigen::Matrix. Number of elements is not a multiple of number of"
                                   " requested rows.", "sim_env/YamlUtils.h");
                    return false;
                }
                matrix.resize(Eigen::NoChange, num_elements / _Rows);
            }
            if (num_elements != (unsigned int)(matrix.cols() * matrix.rows())) {
                logger->logErr("Could not decode Eigen::Matrix. Number of elements in YAML node is not equal to the"
                               " number of requested matrix elements.", "sim_env/YamlUtils.h");
                return false;
            }
            for (unsigned int r = 0; r < matrix.rows(); ++r) {
                for (unsigned int c = 0; c < matrix.cols(); ++c) {
                    matrix(r, c) = node[(int)(r * matrix.cols() + c)].as<_Scalar>();
                }
            }
            return true;
        }
    };

    // Eigen::Array
    template< typename _Scalar, int _Rows, int _Cols>
    struct convert< Eigen::Array< _Scalar, _Rows, _Cols> > {

        static Node encode(const Eigen::Array< _Scalar, _Rows, _Cols>& array) {
            Node node;
            long num_elements = array.cols() * array.rows();
            for (int i = 0; i < num_elements; ++i) {
                node.push_back(array(i / array.cols(), i % array.cols()));
            }
            return node;
        }

        static bool decode(const Node &node, Eigen::Array< _Scalar, _Rows, _Cols> &array) {
            size_t num_elements = node.size();
            sim_env::LoggerPtr logger = sim_env::DefaultLogger::getInstance();
            // The desired dimension is governed by _Rows and _Cols
            if (_Rows == Eigen::Dynamic && _Cols == Eigen::Dynamic) {
                // There is no way to decide what format the matrix should be, so we fail here
                logger->logErr("Could not decode Eigen::Array. _Rows and _Cols can not be dynamic at the same time.",
                               "sim_env/YamlUtils.h");
                return false;
            }

            // Check if we have dynamic rows and specified number of columns
            if (_Rows == Eigen::Dynamic) {
                if (num_elements % _Cols != 0) {
                    logger->logErr("Could not decode Eigen::Array. Number of elements is not a multiple"
                                           " of number of requested columns.", "sim_env/YamlUtils.h");
                    return false;
                }
                array.resize(num_elements / _Cols, Eigen::NoChange);
            } else if (_Cols == Eigen::Dynamic) {
                if (num_elements % _Rows != 0) {
                    logger->logErr("Could not decode Eigen::Array. Number of elements is not a multiple of number of"
                                           " requested rows.", "sim_env/YamlUtils.h");
                    return false;
                }
                array.resize(Eigen::NoChange, num_elements / _Rows);
            }
            if (num_elements != array.cols() * array.rows()) {
                logger->logErr("Could not decode Eigen::Array. Number of elements in YAML node is not equal to the"
                                       " number of requested matrix elements.", "sim_env/YamlUtils.h");
                return false;
            }
            for (unsigned int r = 0; r < array.rows(); ++r) {
                for (unsigned int c = 0; c < array.cols(); ++c) {
                    array(r, c) = node[(int)(r * array.cols() + c)].as<_Scalar>();
                }
            }
            return true;
        }
    };
}

#endif //SIM_ENV_YAMLUTILS_H
