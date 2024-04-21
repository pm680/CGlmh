#version 430 core

// Define a uniform struct for lights
struct Light {
    // The matrices are used for shadow mapping. You need to fill it according to how we are filling it when building the normal maps (node_render_shadow_mapping.cpp). 
    // Now, they are filled with identity matrix. You need to modify C++ code in node_render_deferred_lighting.cpp.
    // Position and color are filled.
    mat4 light_projection;
    mat4 light_view;
    vec3 position;
    float radius;
    vec3 color; // Just use the same diffuse and specular color.
    int shadow_map_id;
};

layout(binding = 0) buffer lightsBuffer {
    Light lights[4];
};

uniform vec2 iResolution;

uniform sampler2D diffuseColorSampler;
uniform sampler2D normalMapSampler; // You should apply normal mapping in rasterize_impl.fs
uniform sampler2D metallicRoughnessSampler;
uniform sampler2DArray shadow_maps;
uniform sampler2D position;

// uniform float alpha;
uniform vec3 camPos;

uniform int light_count;

layout(location = 0) out vec4 Color;

vec3 blinnPhong(vec3 lightColor, vec3 lightPos, vec3 camPos, vec3 pos, vec3 normal, 
                vec3 diffuseColor, float metallic, float roughness, float shadow){
    float ka = 0.1;
    float ks = metallic * 0.8;
    float kd = 1.0 - ks;

    vec3 ambient = ka * lightColor;

    vec3 lightDir = normalize(lightPos - pos);
    float diff = abs(dot(normal, lightDir));
    vec3 diffuse = kd * diff * lightColor;

    vec3 viewDir = normalize(camPos - pos);
    // vec3 reflectDir = reflect(-lightDir, normal);
    vec3 halfwayDir = normalize(lightDir + viewDir);  
    float spec = pow(abs(dot(normal, halfwayDir)), (1 - roughness) * 64.0);
    vec3 specular = ks * spec * lightColor;  

    return (ambient + (diffuse + specular) * (1 - shadow)) * diffuseColor;
}

float shadowCalculation(mat4 lightProjection, mat4 lightView, int lightShadowMapID,
                        vec3 normal, vec3 lightPos, vec3 pos){
    vec4 clipPos = lightProjection * lightView * vec4(pos, 1.0);
    vec3 projCoords = (clipPos.xyz / clipPos.w) * 0.5 + 0.5;

    vec3 lightDir = normalize(lightPos - pos);
    float bias = max(0.05 * (1.0 - dot(normal, lightDir)), 0.005);
    float currentDepth = projCoords.z;
    float closestDepth = texture(shadow_maps, vec3(projCoords.xy, lightShadowMapID)).r;
    float shadow = currentDepth - bias > closestDepth  ? 1.0 : 0.0;
        
    // PCF
    vec2 textSize = 1.0 / textureSize(shadow_maps, 0).xy;
    float shadowPCF = 0.0;
    float sampleCount = 0.0;
    float sampleSize = 16.0;
    for(float x = -sampleSize; x <= sampleSize; ++x){
        for(float y = -sampleSize; y <= sampleSize; ++y){
            sampleCount += 1.0;
            float closestDepth = texture(shadow_maps, 
                vec3(projCoords.xy + vec2(x, y) * textSize, lightShadowMapID)).r;
            shadowPCF += currentDepth - bias > closestDepth ? 1.0 : 0.0;
        }
    }
    shadowPCF /= sampleCount;

    return shadowPCF;
}

void main() {
    vec2 uv = gl_FragCoord.xy / iResolution;

    vec3 pos = texture(position, uv).xyz;
    vec3 normal = texture(normalMapSampler, uv).xyz;

    vec4 metallicRoughness = texture(metallicRoughnessSampler, uv);
    float metallic = metallicRoughness.x;
    float roughness = metallicRoughness.y;

    vec3 diffuseColor = texture(diffuseColorSampler, uv).rgb;

    vec3 lighting = vec3(0.f);
    for(int i = 0; i < light_count; i++) {
        Light light = lights[i];
        mat4 lightProjection = light.light_projection;
        mat4 lightView = light.light_view;
        vec3 lightPos = light.position;
        vec3 lightColor = light.color;
        int lightShadowMapID = light.shadow_map_id;

        float shadow = shadowCalculation(lightProjection, lightView, lightShadowMapID, normal, lightPos, pos);
        lighting += blinnPhong(lightColor, lightPos, camPos, pos, normal, diffuseColor, metallic, roughness, shadow);
    }
    Color = vec4(lighting, 1.0);
}