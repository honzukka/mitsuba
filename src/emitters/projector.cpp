#include <mitsuba/render/scene.h>
#include <mitsuba/hw/basicshader.h>

MTS_NAMESPACE_BEGIN

struct InvCdfVal { int index; Float value; };

class ProjectorEmitter : public Emitter {
    public:
        ProjectorEmitter(const Properties& props) : Emitter(props) {
            m_type |= EOnSurface;
            m_intensity = props.getSpectrum("intensity", Spectrum(1.0f));
            m_texture = new ConstantSpectrumTexture(
                props.getSpectrum("texture", Spectrum::getD65()));

            m_apertureRadius = props.getFloat("apertureRadius", 0.1f);
            if (m_apertureRadius == 0) {
                Log(EWarn, "Can't have a zero aperture radius -- "
                    "setting to %f", Epsilon);
                m_apertureRadius = Epsilon;
            }

            m_focusDistance = props.getFloat("focusDistance", 1.0f);
            m_fov = props.getFloat("fov", 90);
            m_nearClip = props.getFloat("nearClip", 0.01);
            m_farClip = props.getFloat("farClip", 10000);
            m_useImportanceSampling = props.getBoolean("useImportanceSampling", true);
        }

        ProjectorEmitter(Stream* stream, InstanceManager* manager)
            : Emitter(stream, manager) {
            m_texture = static_cast<Texture*>(manager->getInstance(stream));
            m_intensity = Spectrum(stream);

            m_apertureRadius = stream->readFloat();
            m_focusDistance = stream->readFloat();
            m_fov = stream->readFloat();
            m_nearClip = stream->readFloat();
            m_farClip = stream->readFloat();
            m_useImportanceSampling = stream->readBool();

            configure();
        }

        void configure() {
            m_textureResolution = m_texture->getResolution();
            m_invTextureResolution = Vector3f(1.0f / m_textureResolution.x, 1.0f / m_textureResolution.y, 0.0f);
            m_texturePixelCount = m_textureResolution.x * m_textureResolution.y;
            m_aspectRatio = (Float)m_textureResolution.x * m_invTextureResolution.y;

            /**
            * These do the following (in reverse order):
            *
            * 1. Create transform from projector space to [-1,1]x[-1,1]x[0,1] clip
            *    coordinates (not taking account of the aspect ratio yet)
            *
            * 2+3. Translate and scale to shift the clip coordinates into the
            *    range from zero to one, and take the aspect ratio into account.
            */
            m_projectorToSample =
                  Transform::scale(Vector(-0.5f, -0.5f * m_aspectRatio, 1.0f))
                * Transform::translate(Vector(-1.0f, -1.0f / m_aspectRatio, 0.0f))
                * Transform::perspective(m_fov, m_nearClip, m_farClip);
            m_sampleToProjector = m_projectorToSample.inverse();

            m_aperturePdf = 1 / (M_PI * m_apertureRadius * m_apertureRadius);

            /* Precompute some data for importance(). Please
               look at that function for further details */
            Point min(m_sampleToProjector(Point(0, 0, 0))),
                max(m_sampleToProjector(Point(1, 1, 0)));

            AABB2 imageRect;
            imageRect.reset();
            imageRect.reset();
            imageRect.expandBy(Point2(min.x, min.y) / min.z);
            imageRect.expandBy(Point2(max.x, max.y) / max.z);
            m_normalization = 1.0f / imageRect.getVolume();

            if (m_useImportanceSampling)
                computeInverseCdf();
        }

        void serialize(Stream* stream, InstanceManager* manager) const {
            Emitter::serialize(stream, manager);

            manager->serialize(stream, m_texture.get());
            m_intensity.serialize(stream);

            stream->writeFloat(m_apertureRadius);
            stream->writeFloat(m_focusDistance);
            stream->writeFloat(m_fov);
            stream->writeFloat(m_nearClip);
            stream->writeFloat(m_farClip);
            stream->writeBool(m_useImportanceSampling);
        }

        inline void computeInverseCdf() {
            ref<Bitmap> textureBitmap = m_texture->getBitmap();

            // TODO: deal with bitmap types flexibly
            Assert(textureBitmap->getComponentFormat() == Bitmap::EFloat16);
            Assert(textureBitmap->getPixelFormat() == Bitmap::ERGB);
            Assert(textureBitmap->getChannelCount() == 3);

            const int ch = 3;   // number of channels
            const half *textureData = textureBitmap->getFloat16Data();

            Float totalSum = 0.0f;
            for (int i = 0; i < m_texturePixelCount; i++)
                totalSum += textureData[i * ch] + textureData[i * ch + 1] + textureData[i * ch + 2];

            Float cumulativeSum = 0.0f;
            for (int i = 0; i < m_texturePixelCount; i++) {
                Float intensity = textureData[i * ch] + textureData[i * ch + 1] + textureData[i * ch + 2];
                if (intensity > 0.0f) {
                    Float pdf = intensity / totalSum;
                    cumulativeSum += intensity;     // less precision is lost when we accumulate intensities as opposed to pdfs
                    Float cdfVal = cumulativeSum / totalSum;
                    
                    m_textureInverseCdf.push_back({ i, cdfVal });
                    m_pixelSamplePdf.push_back(pdf * m_texturePixelCount);  // pdf alone is never used, so store the full thing instead
                }
                else
                    m_pixelSamplePdf.push_back(0.0f);
            }

            //Assert(std::get<1>(*(m_textureInverseCdf.end() - 1)) == 1.0f);
        }

        inline int importanceSamplePixel(Float sample) const {
            // binary search the CDF^-1 for sample
            // either find the exact value or the nearest larger value
            // the index stored at the found value corresponds to the pixel being importance sampled

            int beginInd = 0;
            int endInd = m_textureInverseCdf.size() - 1;
            int ind = (beginInd + endInd) / 2;
            
            while (beginInd <= endInd) {
                Float cdfVal = m_textureInverseCdf[ind].value;
                if (cdfVal == sample)
                    return m_textureInverseCdf[ind].index;
                if (cdfVal < sample)
                    beginInd = ind + 1;
                else
                    endInd = ind - 1;
                ind = (beginInd + endInd) / 2;
            }

            // exact value was not found, so return the nearest larger one
            
            if (ind < 0)
                return 0;

            Float cdfVal = m_textureInverseCdf[ind].value;
            if ((cdfVal > sample && ind == 0) ||                                            // sample is smaller that the smallest CDF value
                (cdfVal > sample && m_textureInverseCdf[ind - 1].value < sample))    // the nearest larger value was found
                return m_textureInverseCdf[ind].index;
            else                                                                            // the nearest smaller value was found
                return m_textureInverseCdf[ind + 1].index;
        }

        inline Point uniformSamplePixelArea(int pixelInd, const Point2& sample) const {
            // find a uv coordinate of the texture that corresponds to the desired pixel
            //Assert(m_pixelSamplePdf[pixelInd] > 0.0f);
            Point samplePos = Point(
                (Float)(pixelInd % m_textureResolution.x) * m_invTextureResolution.x,
                (Float)(pixelInd / m_textureResolution.x) * m_invTextureResolution.y,
                0.0f
            );

            // uniformly sample pixel area
            Point2 pixelAreaOffset(
                sample.x * m_invTextureResolution.x,
                sample.y * m_invTextureResolution.y
            );
            samplePos.x += pixelAreaOffset.x;
            samplePos.y += pixelAreaOffset.y;

            return samplePos;
        }

        inline int findPixelIndex(const Point2& uv) const {
            // box filter (evaluateTexture() needs to use the same pixel filtering method!!!)

            Point2i pixelCoords(
                math::floorToInt(uv.x * m_textureResolution.x),
                math::floorToInt(uv.y * m_textureResolution.y)
            );
            return pixelCoords.y * m_textureResolution.x + pixelCoords.x;
        }

        inline Spectrum evaluateTexture(const Point2 &uv) const {
            Intersection its;
            its.hasUVPartials = false;
            its.uv = uv;
            return m_texture->eval(its);
        }

        /**
         * \brief Compute the directional sensor response function
         * of the projector multiplied with the cosine foreshortening
         * factor associated with the image plane
         *
         * \param p
         *     A position on the aperture
         *
         * \param d
         *     A normalized direction vector from the aperture position to the
         *     reference point in question (all in local projector space)
         *
         * \param sample
         *     Optional: a pointer to a 2D point data structure, which will be
         *     used to return the fractional pixel position associated with the
         *     reference point
         */
        inline Float uniformImportance(const Point& p, const Vector& d, Point2* sample = NULL) const {
            /* How is this derived? Imagine a hypothetical image plane at a
               distance of d=1 away from the aperture in projector space.

               Then the visible rectangular portion of the plane has the area

                  A = (2 * tan(0.5 * xfov in radians))^2 / aspect

               Perspective transformations of such aligned rectangles produce
               an equivalent scaled (but otherwise undistorted) rectangle
               in screen space. This means that a strategy, which uniformly
               generates samples in screen space has an associated area
               density of 1/A on this rectangle.

               To compute the solid angle density of a sampled point P on
               the rectangle, we can apply the usual measure conversion term:

                  d_omega = 1/A * distance(P, origin)^2 / cos(theta)

               where theta is the angle that the unit direction vector from
               the origin to P makes with the rectangle. Since

                  distance(P, origin)^2 = Px^2 + Py^2 + 1

               and

                  cos(theta) = 1/sqrt(Px^2 + Py^2 + 1),

               we have

                  d_omega = 1 / (A * cos^3(theta))
            */

            Float cosTheta = Frame::cosTheta(d);

            /* Check if the direction points behind the projector */
            if (cosTheta <= 0)
                return 0.0f;

            Float invCosTheta = 1.0f / cosTheta;

            /* Check if the associated pixel is visible */
            Point scr = m_projectorToSample(p
                + d * (m_focusDistance * invCosTheta));
            if (scr.x < 0 || scr.x > 1 ||
                scr.y < 0 || scr.y > 1)
                return 0.0f;

            // populate sample with UV coordinates of the corresponding texture point
            if (sample) {
                sample->x = scr.x;
                sample->y = scr.y;
            }

            return m_normalization * invCosTheta * invCosTheta * invCosTheta;
        }

        Spectrum samplePosition(PositionSamplingRecord& pRec, const Point2& sample,
            const Point2* extra) const {

            const Transform& trafo = m_worldTransform->eval(pRec.time);

            // uniformly sample the aperture
            Point2 aperturePos = warp::squareToUniformDiskConcentric(sample) * m_apertureRadius;

            pRec.p = trafo.transformAffine(Point(aperturePos.x, aperturePos.y, 0.0f));
            pRec.n = trafo(Vector(0.0f, 0.0f, 1.0f));
            pRec.pdf = m_aperturePdf;
            pRec.measure = EArea;

            return m_intensity;
        }

        Spectrum evalPosition(const PositionSamplingRecord& pRec) const {
            return m_intensity * Spectrum((pRec.measure == EArea) ? m_aperturePdf : 0.0f);
        }

        Float pdfPosition(const PositionSamplingRecord& pRec) const {
            return (pRec.measure == EArea) ? m_aperturePdf : 0.0f;
        }

        Spectrum sampleDirection(DirectionSamplingRecord& dRec,
            PositionSamplingRecord& pRec,
            const Point2& sample,
            const Point2* extra) const {

            bool doImportanceSampling = (m_useImportanceSampling && extra != NULL);
            const Transform& trafo = m_worldTransform->eval(pRec.time);

            // now we need to sample the texture and generate a direction
            // based on the aperture sample (in pRec from samplePosition call)
            // and the texture sample

            Point samplePos;
            int pixelInd = -1;
            if (doImportanceSampling) {
                pixelInd = importanceSamplePixel(extra->x);
                samplePos = uniformSamplePixelArea(pixelInd, sample);
            }
            else
                samplePos = Point(sample.x, sample.y, 0.0f);

            pRec.uv = Point2(samplePos.x, samplePos.y);

            /*  Compute the corresponding position on the
                near plane (in local projector space) */
            Point nearP = m_sampleToProjector(samplePos);
            nearP.x = nearP.x * (m_focusDistance / nearP.z);
            nearP.y = nearP.y * (m_focusDistance / nearP.z);
            nearP.z = m_focusDistance;

            Point apertureP = trafo.inverse().transformAffine(pRec.p);

            /* Turn that into a normalized ray direction */
            Vector d = normalize(nearP - apertureP);
            dRec.d = trafo(d);
            dRec.measure = ESolidAngle;
            dRec.pdf = m_normalization / (d.z * d.z * d.z); // conversion to solid angle measure (see importance())
            
            Spectrum result = evaluateTexture(pRec.uv);

            if (doImportanceSampling) {
                dRec.pdf *= m_pixelSamplePdf[pixelInd];
                result /= m_pixelSamplePdf[pixelInd];
            }
            
            return result;
        }

        Float pdfDirection(const DirectionSamplingRecord& dRec,
            const PositionSamplingRecord& pRec) const {

            if (dRec.measure != ESolidAngle)
                return 0.0f;

            Transform invTrafo = m_worldTransform->eval(pRec.time).inverse();
            Point2 uv;
            Float pdf = uniformImportance(invTrafo.transformAffine(pRec.p), invTrafo(dRec.d), &uv);

            if (m_useImportanceSampling)
                pdf *= m_pixelSamplePdf[findPixelIndex(uv)];

            return pdf;
        }

        Spectrum evalDirection(const DirectionSamplingRecord& dRec,
            const PositionSamplingRecord& pRec) const {

            if (dRec.measure != ESolidAngle)
                return Spectrum(0.0f);

            Transform invTrafo = m_worldTransform->eval(pRec.time).inverse();
            Point2 uv;
            Float importanceVal = uniformImportance(invTrafo.transformAffine(pRec.p), invTrafo(dRec.d), &uv);
            
            return evaluateTexture(uv) * importanceVal;
        }

        Spectrum sampleRay(Ray& ray,
            const Point2& pixelSample,
            const Point2& otherSample,
            Float time) const {

            Point2 tmp = warp::squareToUniformDiskConcentric(otherSample)
                * m_apertureRadius;
            ray.time = time;

            /* Compute the corresponding position on the
               near plane (in local projector space) */
            Point nearP = m_sampleToProjector(Point(
                pixelSample.x * m_invTextureResolution.x,
                pixelSample.y * m_invTextureResolution.y, 0.0f));

            /* Aperture position */
            Point apertureP(tmp.x, tmp.y, 0.0f);

            /* Sampled position on the focal plane */
            Point focusP = nearP * (m_focusDistance / nearP.z);

            /* Turn these into a normalized ray direction, and
               adjust the ray interval accordingly */
            Vector d = normalize(focusP - apertureP);
            Float invZ = 1.0f / d.z;
            ray.mint = m_nearClip * invZ;
            ray.maxt = m_farClip * invZ;

            const Transform& trafo = m_worldTransform->eval(ray.time);
            ray.setOrigin(trafo.transformAffine(apertureP));
            ray.setDirection(trafo(d));

            return m_intensity * evaluateTexture(pixelSample);
        }

        Spectrum sampleDirect(DirectSamplingRecord& dRec, const Point2& sample) const {

            const Transform& trafo = m_worldTransform->eval(dRec.time);

            /* Transform the reference point into the local coordinate system */
            Point refP = trafo.inverse().transformAffine(dRec.ref);

            /* Check if it is outside of the clip range */
            if (refP.z < m_nearClip || refP.z > m_farClip) {
                dRec.pdf = 0.0f;
                return Spectrum(0.0f);
            }

            /* Sample a position on the aperture (in local coordinates) */
            Point2 tmp = warp::squareToUniformDiskConcentric(sample)
                * m_apertureRadius;
            Point apertureP(tmp.x, tmp.y, 0);

            /* Compute the normalized direction vector from the
               aperture position to the reference point */
            Vector localD(refP - apertureP);
            Float dist = localD.length(),
                invDist = 1.0f / dist;
            localD *= invDist;

            Float value = uniformImportance(apertureP, localD, &dRec.uv);
            if (value == 0.0f) {
                dRec.pdf = 0.0f;
                return Spectrum(0.0f);
            }

            dRec.p = trafo.transformAffine(apertureP);
            dRec.d = (dRec.p - dRec.ref) * invDist;
            dRec.dist = dist;
            dRec.n = trafo(Vector(0.0f, 0.0f, 1.0f));
            dRec.pdf = m_aperturePdf * dist * dist / (Frame::cosTheta(localD));
            dRec.measure = ESolidAngle;

            /* intentionally missing a cosine factor wrt. the aperture
               disk (it is already accounted for in importance()) */
            return m_intensity * evaluateTexture(dRec.uv) * Spectrum(value * invDist * invDist);
        }

        Float pdfDirect(const DirectSamplingRecord& dRec) const {
            Float dp = -dot(dRec.n, dRec.d);
            if (dp < 0)
                return 0.0f;

            if (dRec.measure == ESolidAngle)
                return m_aperturePdf * dRec.dist * dRec.dist / dp;
            else if (dRec.measure == EArea)
                return m_aperturePdf;
            else
                return 0.0f;
        }

        void addChild(const std::string& name, ConfigurableObject* child) {
            if (child->getClass()->derivesFrom(MTS_CLASS(Texture)) && name == "texture") {
                m_texture = static_cast<Texture*>(child);
            }
            else {
                Emitter::addChild(name, child);
            }
        }

        AABB getAABB() const {
            return m_worldTransform->getTranslationBounds();
        }

        std::string toString() const {
            std::ostringstream oss;
            oss << "ProjectorEmitter[" << std::endl
                << "  intensity = " << m_intensity.toString() << "," << std::endl
                << "  texture = " << m_texture.toString() << "," << std::endl
                << "  apertureRadius = " << m_apertureRadius << ", " << std::endl
                << "  focusDistance = " << m_focusDistance << ", " << std::endl
                << "  fov = " << m_fov << ", " << std::endl
                << "  nearClip = " << m_nearClip << ", " << std::endl
                << "  farClip = " << m_farClip << ", " << std::endl
                << "  useImportanceSampling = " << m_useImportanceSampling << std::endl
                << "]";
            return oss.str();
        }

        Shader* createShader(Renderer* renderer) const;

        MTS_DECLARE_CLASS()
    private:
        Spectrum m_intensity;
        ref<Texture> m_texture;

        Float m_apertureRadius;
        Float m_focusDistance;
        Float m_fov;
        Float m_nearClip, m_farClip;
        bool m_useImportanceSampling;
        
        Float m_aperturePdf;
        Vector3i m_textureResolution;
        Vector3f m_invTextureResolution;
        int m_texturePixelCount;
        Float m_aspectRatio;
        Transform m_projectorToSample;
        Transform m_sampleToProjector;
        Float m_normalization;

        std::vector<InvCdfVal> m_textureInverseCdf;
        std::vector<Float> m_pixelSamplePdf;
};

// ================ Hardware shader implementation ================

// TODO: implement the projector in the preview as well

class ProjectorEmitterShader : public Shader {
public:
    ProjectorEmitterShader(Renderer* renderer, Transform worldToEmitter, const Texture* texture)
        : Shader(renderer, EEmitterShader), m_worldToEmitter(worldToEmitter), m_texture(texture) {
        m_textureShader = renderer->registerShaderForResource(m_texture.get());
    }

    bool isComplete() const {
        return m_textureShader.get() != NULL;
    }

    void cleanup(Renderer* renderer) {
        renderer->unregisterShaderForResource(m_texture.get());
    }

    void putDependencies(std::vector<Shader*>& deps) {
        deps.push_back(m_textureShader.get());
    }

    void generateCode(std::ostringstream& oss, const std::string& evalName,
        const std::vector<std::string>& depNames) const {
        // do not influence the scene appearance in any way
        // TODO: what does this do actually? :D
        oss << "uniform mat4 " << evalName << "_worldToEmitter;" << endl
            << "vec3 " << evalName << "_dir(vec3 wo) {" << endl
            << "      return 0.0;"
            << "}" << endl;
    }

    void resolve(const GPUProgram* program, const std::string& evalName, std::vector<int>& parameterIDs) const {
        parameterIDs.push_back(program->getParameterID(evalName + "_worldToEmitter", false));
    }

    void bind(GPUProgram* program, const std::vector<int>& parameterIDs, int& textureUnitOffset) const {
        program->setParameter(parameterIDs[0], m_worldToEmitter);
    }

    MTS_DECLARE_CLASS()
private:
    Transform m_worldToEmitter;
    Float m_invTransitionWidth;
    Float m_cutoffAngle, m_cosCutoffAngle;
    Float m_cosBeamWidth, m_uvFactor;
    ref<const Texture> m_texture;
    ref<Shader> m_textureShader;
};

Shader* ProjectorEmitter::createShader(Renderer* renderer) const {
    const Transform& trafo = m_worldTransform->eval(0.0f);

    return new ProjectorEmitterShader(renderer, trafo.inverse(), m_texture.get());
}

MTS_IMPLEMENT_CLASS(ProjectorEmitterShader, false, Shader)
MTS_IMPLEMENT_CLASS_S(ProjectorEmitter, false, Emitter)
MTS_EXPORT_PLUGIN(ProjectorEmitter, "Projector");
MTS_NAMESPACE_END
