/*
    This file is part of Mitsuba, a physically based rendering system.

    Copyright (c) 2007-2011 by Wenzel Jakob and others.

    Mitsuba is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Mitsuba is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <mitsuba/hw/viewer.h>
#include <mitsuba/hw/gputexture.h>
#include <mitsuba/hw/gpuprogram.h>
#include <mitsuba/hw/shadow.h>
#include <mitsuba/render/scene.h>
#include <mitsuba/core/bitmap.h>
#include <mitsuba/core/fresolver.h>
#include <mitsuba/core/fstream.h>
#include <mitsuba/hw/shadow.h>

#if defined(__APPLE__)
# include <OpenGL/glew.h>
#else
# include <GL/glew.h>
#endif

MTS_NAMESPACE_BEGIN

#define type ShadowMapGenerator::EDirectional

class VPLPlayground : public Viewer {
public:
	VPLPlayground() {
		FileResolver *fResolver = Thread::getThread()->getFileResolver();
		fResolver->appendPath("scenes/veach_bidir");
		m_scene = loadScene("scenes/veach_bidir/bidir.xml");
		m_scene->configure();
		m_scene->initialize();
	}

	bool init(int argc, char **argv) {
		const std::vector<TriMesh *> &meshes = m_scene->getMeshes();

		Matrix4x4 identity;
		identity.setIdentity();
		for (size_t i=0; i<meshes.size(); ++i)
			m_geometry.push_back(std::make_pair(
				m_renderer->registerGeometry(meshes[i]), identity));

		m_shadowGen = new ShadowMapGenerator(m_renderer);
		m_shadowGen->init();
		m_shadowMap = m_shadowGen->allocate(m_renderer, type, 512);
		m_shadowMap->init();

		return true;
	}

	void draw() {
		ProjectiveCamera *camera = static_cast<ProjectiveCamera *>(m_scene->getSensor());
		Transform trafo = camera->getViewTransform(0);

		AABB aabb = m_scene->getKDTree()->getAABB();
		Vector d = trafo(Vector(0, 0, 1));
		Transform dir = m_shadowGen->directionalFindGoodFrame(aabb, d);

		m_renderer->debugString("Starting to render VPL..");
		m_shadowGen->render(m_renderer, m_shadowMap, type, dir, 0.01, 10, m_geometry);
		m_renderer->debugString("Done rendering VPL..");

		{
			ref<Bitmap> bitmap = new Bitmap(Bitmap::ELuminance, Bitmap::EFloat32, Vector2i(512));
			m_shadowMap->download(bitmap);
			ref<FileStream> fs = new FileStream("out.pfm", FileStream::ETruncReadWrite);
			bitmap->write(Bitmap::EPFM, fs);
		}

		glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
		glDisable(GL_LIGHTING);
		glDisable(GL_BLEND);
		glEnable(GL_DEPTH_TEST);
		glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

		Transform perspective = Transform::glPerspective(85, 0.1, 100);
	
		m_renderer->setCamera(perspective.getMatrix(), trafo.getMatrix());

		{
			Matrix4x4 currentObjTrafo;
			currentObjTrafo.setIdentity();
			m_renderer->pushTransform(trafo);
			m_renderer->beginDrawingMeshes();
			for (std::vector<Renderer::TransformedGPUGeometry>::const_iterator it = m_geometry.begin(); 
					it != m_geometry.end(); ++it) {
				const GPUGeometry *geo = (*it).first;
				const Matrix4x4 &trafo = (*it).second;

				if (trafo != currentObjTrafo) {
					m_renderer->popTransform();
					m_renderer->pushTransform(trafo);
					currentObjTrafo = trafo;
				}

				m_renderer->drawMesh(geo);
			}
			m_renderer->popTransform();
			m_renderer->endDrawingMeshes();
		}
	}

	MTS_DECLARE_UTILITY()
private:
	std::vector<Renderer::TransformedGPUGeometry> m_geometry;
	ref<ShadowMapGenerator> m_shadowGen;
	ref<GPUTexture> m_shadowMap;
	ref<Scene> m_scene;
};

MTS_EXPORT_UTILITY(VPLPlayground, "Nonlinear projection test");
MTS_NAMESPACE_END
