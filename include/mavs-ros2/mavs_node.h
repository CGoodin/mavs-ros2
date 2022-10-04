

class MavsNode : public rclcpp::Node{
  public:
	MavsNode(): Node("mavs_node"){
		// base node
	}

  protected:
	
	float GetFloatParam(std::string param_name, float default_value){
		this->declare_parameter(param_name, default_value);
		float val = default_value;
		if (this->has_parameter(param_name)){
			val = this->get_parameter(param_name).as_double();
		}
		return val;
	}

	int GetIntParam(std::string param_name, int default_value){
		this->declare_parameter(param_name, default_value);
		int val = default_value;
		if (this->has_parameter(param_name)){
			val = this->get_parameter(param_name).as_int();
		}
		return val;
	}

	std::vector<std::string> GetStringArrayParam(std::string param_name, std::vector<std::string> default_value){
		this->declare_parameter(param_name, default_value);
		std::vector<std::string> val = default_value;
		if (this->has_parameter(param_name)){
			val = this->get_parameter(param_name).as_string_array();
		}
		return val;
	}

	std::vector<float> GetFloatArrayParam(std::string param_name, std::vector<float> default_value){
		this->declare_parameter(param_name, default_value);
		std::vector<float> val = default_value;
		std::vector<double> tmpval;
		if (this->has_parameter(param_name)){
			tmpval = this->get_parameter(param_name).as_double_array();
			val.clear();
			for (int i=0;i<(int)tmpval.size();i++){
				val.push_back((float)tmpval[i]);
			}
		}
		return val;
	}

	bool GetBoolParam(std::string param_name, bool default_value){
		this->declare_parameter(param_name, default_value);
		bool val = default_value;
		if (this->has_parameter(param_name)){
			val = this->get_parameter(param_name).as_bool();
		}
		return val;
	}

	std::string GetStringParam(std::string param_name, std::string default_value){
		this->declare_parameter(param_name, default_value);
		std::string val = default_value;
		if (this->has_parameter(param_name)){
			val = this->get_parameter(param_name).as_string();
		}
		return val;
	}

};
