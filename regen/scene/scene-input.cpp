/*
 * scene-input.cpp
 *
 *  Created on: Nov 3, 2013
 *      Author: daniel
 */

#include "scene-input.h"

using namespace regen::scene;
using namespace regen;
using namespace std;

#include <regen/utility/string-util.h>

static void pushIndexToSequence(
		GLuint numIndices, list<GLuint> &indices, GLuint index) {
	if (index >= numIndices) {
		REGEN_WARN("Invalid index: " << index << ">=" << numIndices << ".");
		return;
	}
	indices.push_back(index);
}

////////////////
////////////////

ref_ptr<SceneInputNode> SceneInput::getNode(
		const string &nodeCategory,
		const string &nodeName) {
	return getRoot()->getFirstChild(nodeCategory, nodeName);
}

////////////////
////////////////

SceneInputNode::SceneInputNode(SceneInputNode *parent)
		: parent_(parent) {
}

string SceneInputNode::getDescription() {
	stringstream ss;

	ss << '<' << getCategory() << ' ';
	ss << "id=" << getName() << ' ';
	const map<string, string> &atts = getAttributes();
	for (const auto & att : atts) {
		if (att.first == "id") continue;
		ss << att.first << '=' << att.second << ' ';
	}
	ss << '>';

	return ss.str();
}

list<GLuint> SceneInputNode::getIndexSequence(GLuint numIndices) {
	list<GLuint> indices;
	if (hasAttribute("index")) {
		pushIndexToSequence(numIndices, indices, getValue<GLint>("index", 0u));
	} else if (hasAttribute("from-index") || hasAttribute("to-index")) {
		auto from = getValue<GLuint>("from-index", 0u);
		auto to = getValue<GLuint>("to-index", numIndices - 1);
		auto step = getValue<GLuint>("index-step", 1u);
		for (GLuint i = from; i <= to; i += step) {
			pushIndexToSequence(numIndices, indices, i);
		}
	} else if (hasAttribute("indices")) {
		const auto indicesAtt = getValue<string>("indices", "0");
		vector<string> indicesStr;
		boost::split(indicesStr, indicesAtt, boost::is_any_of(","));
		for (auto & it : indicesStr)
			pushIndexToSequence(numIndices, indices, atoi(it.c_str()));
	} else if (hasAttribute("random-indices")) {
		auto indexCount = getValue<GLuint>("random-indices", numIndices);
		while (indexCount > 0) {
			--indexCount;
			pushIndexToSequence(numIndices, indices, rand() % numIndices);
		}
	}

	if (indices.empty()) {
		// If no indices found, add all
		for (GLuint i = 0; i < numIndices; i += 1) {
			pushIndexToSequence(numIndices, indices, i);
		}
	}
	return indices;
}

list<ref_ptr<SceneInputNode> > SceneInputNode::getChildren(const std::string &category) {
	const list<ref_ptr<SceneInputNode> > &children = getChildren();
	list<ref_ptr<SceneInputNode> > out;
	for (const auto& n : children) {
		if (n->getCategory() == category) out.push_back(n);
	}
	return out;
}

list<ref_ptr<SceneInputNode> > SceneInputNode::getChildren(const string &category, const string &name) {
	const list<ref_ptr<SceneInputNode> > &children = getChildren();
	list<ref_ptr<SceneInputNode> > out;
	for (const auto& n : children) {
		if (n->getName() == name && n->getCategory() == category)
			out.push_back(n);
	}
	return out;
}

ref_ptr<SceneInputNode> SceneInputNode::getFirstChild(const string &category, const string &name) {
	const list<ref_ptr<SceneInputNode> > &children = getChildren();
	list<ref_ptr<SceneInputNode> > out;
	for (const auto& n : children) {
		if (n->getName() == name && n->getCategory() == category)
			return n;
	}
	return {};
}

ref_ptr<SceneInputNode> SceneInputNode::getFirstChild(const string &category) {
	const list<ref_ptr<SceneInputNode> > &children = getChildren();
	list<ref_ptr<SceneInputNode> > out;
	for (const auto& n : children) {
		if (n->getCategory() == category) return n;
	}
	return {};
}

bool SceneInputNode::hasAttribute(const string &key) {
	return getAttributes().count(key) > 0;
}

const string &SceneInputNode::getValue(const string &key) {
	static string emptyString;

	auto needle = getAttributes().find(key);
	if (needle != getAttributes().end()) {
		return needle->second;
	} else {
		if (parent_ != nullptr) {
			return parent_->getValue(key);
		} else {
			return emptyString;
		}
	}
}
