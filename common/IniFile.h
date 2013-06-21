//
// Copyright (c) 2009-2011  Shuntaro Yamazaki (shun-yamazaki (at) aist.go.jp) 
//  and the National Institute of Advanced Industrial Science and Technology   
//
// $Id: ImageMagickIO.h 4304 2010-12-07 09:02:51Z shun $
//

#pragma once

#include <stdexcept>

namespace slib {

class CIniFile
{
private:
	typedef std::map<std::string,std::string> key_list;
	typedef std::map<std::string,key_list> section_list;

public:
	CIniFile() {}
	~CIniFile() {}

	void Load(const std::string& filename) 
	{
		FILE *fr=fopen(filename.c_str(),"rb");
		if (!fr)
			throw std::runtime_error(format("failed to open %s", filename.c_str()));
		TRACE("ini <= %s\n", filename.c_str()); 

		m_data.clear();
		section_list::iterator it = m_data.begin();

		char buf[BUFSIZ];
		while (fgets(buf,BUFSIZ,fr)) 
		{
			// ignore comments
			char *p1 = strchr(buf,';');
			if (p1) *p1=0; 
			p1 = DeleteSpaces(buf);
			if (!strlen(p1)) continue;

			if (p1[0] == '[') {
				p1++;
				char *p2 = strrchr(p1,']');
				if (!p2) {
					TRACE("error: could not find a closing ']'. (ignored)\n");
					continue;
				}
				*p2=0;
				it = InsertSection(p1);
			} else {
				char *p2 = strchr(p1,'=');
				if (!p2) {
					TRACE("error: could not find an equal symbol. (ignored)\n");
					continue;
				}
				*p2=0;
				p2++;
				p1 = DeleteSpaces(p1);
				p2 = DeleteSpaces(p2);
				InsertValue(it,p1,p2);
			}
		}
		fclose(fr);
	}

	int GetInt(const std::string& section, const std::string& key) const {
		const std::string& s = GetString(section,key);
		return atoi(s.c_str());
	}

	double GetFloat(const std::string& section, const std::string& key) const {
		const std::string& s = GetString(section,key);
		return atof(s.c_str());
	}

	bool GetBool(const std::string& section, const std::string& key) const {
		const std::string& s = GetString(section,key);
		if (!s.compare("0") || 
			!s.compare("false") || 
			!s.compare("FALSE"))
			return false;
		else
			return true;
	}

	const std::string& GetString(const std::string& section, const std::string& key) const {
		section_list::const_iterator it1 = m_data.find(section);
		if (it1 == m_data.end())
			throw std::runtime_error(format("could not find a section: %s", section.c_str()));
		key_list::const_iterator it2 = it1->second.find(key);
		if (it2 == it1->second.end())
			throw std::runtime_error(format("could not find a key: %s", key.c_str()));
		return it2->second;
	}

	void Dump() const {
		section_list::const_iterator section = m_data.begin();
		for (; section != m_data.end(); ++section) {
	//		TRACE("[%s]\n",section->first.c_str());
			key_list::const_iterator key = section->second.begin();
			for (; key != section->second.end(); ++key) {
	//			TRACE("%s = %s\n",key->first.c_str(),key->second.c_str());
			}
		}
	}

private:
	char *DeleteSpaces(char *p) {
		while (isspace(p[0]))
			p++;
		char *e = p + strlen(p);
		while (e != p && isspace(e[-1])) {
			e[-1]=0;
			e--;
		}
		return p;
	}

	section_list::iterator InsertSection(const std::string& section_name) {
		key_list new_section;
		return m_data.insert(m_data.begin(),std::make_pair(section_name,new_section));
	}

	void InsertValue(section_list::iterator& it,
		const std::string& key,
		const std::string& val) {
		it->second.insert(std::make_pair(key,val));
	}

private:
	std::map< std::string,key_list> m_data;
};

} // namespace slib
