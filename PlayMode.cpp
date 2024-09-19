#include "PlayMode.hpp"

#include "LitColorTextureProgram.hpp"

#include "DrawLines.hpp"
#include "Load.hpp"
#include "Mesh.hpp"
#include "data_path.hpp"
#include "gl_errors.hpp"

#include <glm/gtc/type_ptr.hpp>

#include <iterator>
#include <random>

GLuint hexapod_meshes_for_lit_color_texture_program = 0;
Load<MeshBuffer> hexapod_meshes(LoadTagDefault, []() -> MeshBuffer const * {
  MeshBuffer const *ret = new MeshBuffer(data_path("edge.pnct"));
  hexapod_meshes_for_lit_color_texture_program =
      ret->make_vao_for_program(lit_color_texture_program->program);
  return ret;
});

Load<Scene> hexapod_scene(LoadTagDefault, []() -> Scene const * {
  return new Scene(data_path("edge.scene"), [&](Scene &scene,
                                                Scene::Transform *transform,
                                                std::string const &mesh_name) {
    Mesh const &mesh = hexapod_meshes->lookup(mesh_name);

    std::cout << "Mesh name: " << mesh_name << std::endl;

    scene.drawables.emplace_back(transform);
    Scene::Drawable &drawable = scene.drawables.back();

    // print position
    std::cout << "Position: " << transform->position.x << " "
              << transform->position.y << " " << transform->position.z
              << std::endl;

    drawable.pipeline = lit_color_texture_program_pipeline;

    drawable.pipeline.vao = hexapod_meshes_for_lit_color_texture_program;
    drawable.pipeline.type = mesh.type;
    drawable.pipeline.start = mesh.start;
    drawable.pipeline.count = mesh.count;
  });
});

Load<Sound::Sample>
dusty_floor_sample(LoadTagDefault, []() -> Sound::Sample const * {
  return new Sound::Sample(data_path("bgm.wav"));
});

Load<Sound::Sample>
warning_sample(LoadTagDefault, []() -> Sound::Sample const * {
  return new Sound::Sample(data_path("warning.wav"));
});

PlayMode::PlayMode() : scene(*hexapod_scene) {
  // get pointers to leg for convenience:

  // get pointer to camera for convenience:
  if (scene.cameras.size() != 1)
    throw std::runtime_error(
        "Expecting scene to have exactly one camera, but it has " +
        std::to_string(scene.cameras.size()));
  camera = &scene.cameras.front();

  // start music loop playing:
  // (note: position will be over-ridden in update())
  leg_tip_loop =
      Sound::loop_3D(*dusty_floor_sample, 1.0f, get_leg_tip_position(), 10.0f);
}

PlayMode::~PlayMode() {}

bool PlayMode::handle_event(SDL_Event const &evt,
                            glm::uvec2 const &window_size) {

  if (evt.type == SDL_KEYDOWN) {
    if (evt.key.keysym.sym == SDLK_ESCAPE) {
      SDL_SetRelativeMouseMode(SDL_FALSE);
      return true;
    } else if (evt.key.keysym.sym == SDLK_a) {
      left.downs += 1;
      left.pressed = true;
      return true;
    } else if (evt.key.keysym.sym == SDLK_d) {
      right.downs += 1;
      right.pressed = true;
      return true;
    } else if (evt.key.keysym.sym == SDLK_w) {
      up.downs += 1;
      up.pressed = true;
      return true;
    } else if (evt.key.keysym.sym == SDLK_s) {
      down.downs += 1;
      down.pressed = true;
      return true;
    } else if (evt.key.keysym.sym == SDLK_SPACE) {
      space.pressed = true;
      return true;
    } else if (evt.key.keysym.sym == SDLK_q) {
      q_pressed = true;
      return true;
    } else if (evt.key.keysym.sym == SDLK_e) {
      e_pressed = true;
      return true;
    }
  } else if (evt.type == SDL_KEYUP) {
    if (evt.key.keysym.sym == SDLK_a) {
      left.pressed = false;
      return true;
    } else if (evt.key.keysym.sym == SDLK_d) {
      right.pressed = false;
      return true;
    } else if (evt.key.keysym.sym == SDLK_w) {
      up.pressed = false;
      return true;
    } else if (evt.key.keysym.sym == SDLK_s) {
      down.pressed = false;
      return true;
    } else if (evt.key.keysym.sym == SDLK_SPACE) {
      space.pressed = false;
      return true;
    }
  } else if (evt.type == SDL_MOUSEBUTTONDOWN) {
    if (SDL_GetRelativeMouseMode() == SDL_FALSE) {
      SDL_SetRelativeMouseMode(SDL_TRUE);
      return true;
    }
  } else if (evt.type == SDL_MOUSEMOTION) {
    if (SDL_GetRelativeMouseMode() == SDL_TRUE) {
      glm::vec2 motion = glm::vec2(evt.motion.xrel / float(window_size.y),
                                   -evt.motion.yrel / float(window_size.y));
      camera->transform->rotation = glm::normalize(
          camera->transform->rotation *
          glm::angleAxis(-motion.x * camera->fovy,
                         glm::vec3(0.0f, 1.0f, 0.0f)) *
          glm::angleAxis(motion.y * camera->fovy, glm::vec3(1.0f, 0.0f, 0.0f)));
      return true;
    }
  }

  if (space.pressed) {
    // leg_tip_loop->stop(1.0f / 60.0f);
    if (leg_tip_loop->volume.value > 0.0f) {
      leg_tip_loop->volume.target = 0.0f;
      leg_tip_loop->volume.ramp = 1.0f / 60.0f;
    } else {
      leg_tip_loop->volume.target = 1.0f;
      leg_tip_loop->volume.ramp = 1.0f / 60.0f;
    }
    // print out volume
    std::cout << "Volume: " << leg_tip_loop->volume.value << std::endl;
  }

  return false;
}

void PlayMode::update(float elapsed) {
  total_time += elapsed;
  if (total_time > 5.0f) {
    // randomly pick from { 6, 7, 8, 9 }
    int index = rand() % 4 + 6;
    total_time -= 5.0f;
    if (index == 6) {
      move1 = true;
      warning_once = Sound::play_3D(
          *warning_sample, 2.0f, scene.drawables[6].transform->position, 10.0f);
    } else if (index == 7) {
      move2 = true;
      warning_once = Sound::play_3D(
          *warning_sample, 2.0f, scene.drawables[7].transform->position, 10.0f);
    } else if (index == 8) {
      move3 = true;
      warning_once = Sound::play_3D(
          *warning_sample, 2.0f, scene.drawables[8].transform->position, 10.0f);
    } else {
      move4 = true;
      warning_once = Sound::play_3D(
          *warning_sample, 2.0f, scene.drawables[9].transform->position, 10.0f);
    }
  }

  // music play
  {

  }

  // camera lookAt update
  {
    if (q_pressed) {
      angle += 90;
      if (angle >= 360) {
        angle -= 360;
      }
      q_pressed = false;
    }
    if (e_pressed) {
      angle -= 90;
      if (angle < 0) {
        angle += 360;
      }
      e_pressed = false;
    }
    float rd = static_cast<float>(angle);
    glm::vec3 lookAt =
        glm::vec3(sin(glm::radians(rd)), cos(glm::radians(rd)), 0.0f);
    glm::vec3 lookRight = glm::normalize(glm::vec3(-lookAt.y, lookAt.x, 0.0f));
    glm::vec3 lookUp = glm::cross(lookAt, lookRight);
    glm::mat3 rotationMatrix(lookRight, lookUp, lookAt);
    camera->transform->rotation = glm::quat_cast(rotationMatrix);
  }

  // lookAt close move
  {
    if (angle == 0) {
      move3 = false;
    } else if (angle == 90) {
      move4 = false;
    } else if (angle == 180) {
      move2 = false;
    } else if (angle == 270) {
      move1 = false;
    }
  }

  // update move
  {
    if (move1) {
      // if (first) {
      //   first = false;
      //   warning_once =
      //       Sound::play_3D(*warning_sample, 2.0f,
      //                      scene.drawables[6].transform->position, 10.0f);
      // } else {
      // }
      scene.drawables[6].transform->position +=
          glm::vec3(-5.0f, 0.0f, 0.0f) * elapsed;
    }
    if (move2) {
      scene.drawables[7].transform->position +=
          glm::vec3(0.0f, -5.0f, 0.0f) * elapsed;
    }
    if (move3) {
      scene.drawables[8].transform->position +=
          glm::vec3(0.0f, 5.0f, 0.0f) * elapsed;
    }
    if (move4) {
      scene.drawables[9].transform->position +=
          glm::vec3(5.0f, 0.0f, 0.0f) * elapsed;
    }
  }

  // move sound to follow leg tip position:
  leg_tip_loop->set_position(glm::vec3(0.0f, 0.0f, 10.0f), 1.0f / 60.0f);

  { // update listener to camera position:
    glm::mat4x3 frame = camera->transform->make_local_to_parent();
    glm::vec3 frame_right = frame[0];
    glm::vec3 frame_at = frame[3];
    Sound::listener.set_position_right(frame_at, frame_right, 1.0f / 60.0f);
  }
}

void PlayMode::draw(glm::uvec2 const &drawable_size) {
  // update camera aspect ratio for drawable:
  camera->aspect = float(drawable_size.x) / float(drawable_size.y);

  // set up light type and position for lit_color_texture_program:
  // TODO: consider using the Light(s) in the scene to do this
  glUseProgram(lit_color_texture_program->program);
  glUniform1i(lit_color_texture_program->LIGHT_TYPE_int, 1);
  glUniform3fv(lit_color_texture_program->LIGHT_DIRECTION_vec3, 1,
               glm::value_ptr(glm::vec3(0.0f, 0.0f, -1.0f)));
  glUniform3fv(lit_color_texture_program->LIGHT_ENERGY_vec3, 1,
               glm::value_ptr(glm::vec3(1.0f, 1.0f, 0.95f)));
  glUseProgram(0);

  glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
  glClearDepth(1.0f); // 1.0 is actually the default value to clear the depth
                      // buffer to, but FYI you can change it.
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS); // this is the default depth comparison function, but
                        // FYI you can change it.

  scene.draw(*camera);

  { // use DrawLines to overlay some text:
    glDisable(GL_DEPTH_TEST);
    float aspect = float(drawable_size.x) / float(drawable_size.y);
    DrawLines lines(glm::mat4(1.0f / aspect, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
                              0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                              1.0f));

    constexpr float H = 0.09f;
    lines.draw_text(
        "Hear sound in back, use Q/E to rotate camera and look At them",
        glm::vec3(-aspect + 0.1f * H, -1.0 + 0.1f * H, 0.0),
        glm::vec3(H, 0.0f, 0.0f), glm::vec3(0.0f, H, 0.0f),
        glm::u8vec4(0x00, 0x00, 0x00, 0x00));
    float ofs = 2.0f / drawable_size.y;
    lines.draw_text(
        "Hear sound in back, use Q/E to rotate camera and look At them",
        glm::vec3(-aspect + 0.1f * H + ofs, -1.0 + +0.1f * H + ofs, 0.0),
        glm::vec3(H, 0.0f, 0.0f), glm::vec3(0.0f, H, 0.0f),
        glm::u8vec4(0xff, 0xff, 0xff, 0x00));
  }
  GL_ERRORS();
}

glm::vec3 PlayMode::get_leg_tip_position() {
  // the vertex position here was read from the model in blender:
  return glm::vec3(-1.26137f, -11.861f, 0.0f);
}
