/*
 * CRISP: Contact-Rich Simulation Platform
 * Copyright (c) 2026 The CRISP authors.
 * Licensed for academic and non-commercial use only.
 * See the LICENSE file for details.
 */

#pragma once  // clang-format off

// bump-allocated fields of model_t
#define CRISP_MODEL_BUMP(X, SIZE)                                              \
  X ( packed, real_t,           state, q0,            SIZE(nq)               ) \
                                                                               \
  X ( vector, int,              body,  nameid,        SIZE(nbody)            ) \
  X ( vector, int,              body,  parentid,      SIZE(nbody)            ) \
  X ( vector, bool,             body,  fixed,         SIZE(nbody)            ) \
  X ( vector, Vector3r,         body,  pos,           SIZE(nbody)            ) \
  X ( vector, Quaternionr,      body,  quat,          SIZE(nbody)            ) \
  X ( vector, Vector3r,         body,  ipos,          SIZE(nbody)            ) \
  X ( vector, Quaternionr,      body,  iquat,         SIZE(nbody)            ) \
  X ( vector, real_t,           body,  mass,          SIZE(nbody)            ) \
  X ( vector, inertia_t,        body,  inertia,       SIZE(nbody)            ) \
                                                                               \
  X ( vector, joint_e,          jnt,   type,          SIZE(nbody)            ) \
  X ( vector, int,              jnt,   nameid,        SIZE(nbody)            ) \
  X ( vector, int,              jnt,   qadr,          SIZE(nbody)            ) \
  X ( vector, int,              jnt,   vadr,          SIZE(nbody)            ) \
                                                                               \
  X ( vector, int,              dof,   bodyid,        SIZE(nv)               ) \
  X ( vector, int,              dof,   parentid,      SIZE(nv)               ) \
  X ( vector, int,              dof,   parentdof,     SIZE(nv)               ) \
  X ( vector, int,              dof,   Madr,          SIZE(nv)               ) \
  X ( vector, Vector3r,         dof,   axis,          SIZE(nv)               ) \
  X ( packed, real_t,           dof,   imp,           SIZE(nv)               ) \
                                                                               \
  X ( vector, int,              damp,  vadr,          SIZE(ndamp)            ) \
  X ( vector, real_t,           damp,  coeff,         SIZE(ndamp)            ) \
                                                                               \
  X ( vector, int,              fric,  vadr,          SIZE(nfric)            ) \
  X ( vector, real_t,           fric,  value,         SIZE(nfric)            ) \
  X ( vector, real_t,           fric,  bound,         SIZE(nfric)            ) \
                                                                               \
  X ( vector, int,              lim,   qadr,          SIZE(nlim)             ) \
  X ( vector, int,              lim,   vadr,          SIZE(nlim)             ) \
  X ( vector, Vector2r,         lim,   range,         SIZE(nlim)             ) \
                                                                               \
  X ( vector, geometry_e,       geom,  type,          SIZE(ngeom)            ) \
  X ( vector, int,              geom,  nameid,        SIZE(ngeom)            ) \
  X ( vector, int,              geom,  bodyid,        SIZE(ngeom)            ) \
  X ( vector, int,              geom,  assetid,       SIZE(ngeom)            ) \
  X ( vector, int,              geom,  matid,         SIZE(ngeom)            ) \
  X ( vector, Vector3r,         geom,  pos,           SIZE(ngeom)            ) \
  X ( vector, Quaternionr,      geom,  quat,          SIZE(ngeom)            ) \
  X ( vector, Vector4r,         geom,  size,          SIZE(ngeom)            ) \
  X ( vector, bool,             geom,  visual,        SIZE(ngeom)            ) \
  X ( vector, uint8_t,          geom,  con_type,      SIZE(ngeom)            ) \
  X ( vector, uint8_t,          geom,  con_affinity,  SIZE(ngeom)            ) \
  X ( vector, real_t,           geom,  mu,            SIZE(ngeom)            ) \
  X ( vector, real_t,           geom,  k,             SIZE(ngeom)            ) \
                                                                               \
  X ( vector, int,              mesh,  nameid,        SIZE(nmesh)            ) \
  X ( vector, int,              mesh,  pathid,        SIZE(nmesh)            ) \
  X ( vector, int,              mesh,  nvert,         SIZE(nmesh)            ) \
  X ( vector, int,              mesh,  nnormal,       SIZE(nmesh)            ) \
  X ( vector, int,              mesh,  ntexcoord,     SIZE(nmesh)            ) \
  X ( vector, int,              mesh,  nface,         SIZE(nmesh)            ) \
  X ( vector, int,              mesh,  nmat,          SIZE(nmesh)            ) \
  X ( vector, int,              mesh,  matid0,        SIZE(nmesh)            ) \
  X ( vector, Matrix3Xf,        mesh,  vert,          SIZE(nmesh)            ) \
  X ( vector, Matrix3Xf,        mesh,  normal,        SIZE(nmesh)            ) \
  X ( vector, Matrix2Xf,        mesh,  texcoord,      SIZE(nmesh)            ) \
  X ( vector, Matrix3Xi,        mesh,  face_vert,     SIZE(nmesh)            ) \
  X ( vector, Matrix3Xi,        mesh,  face_normal,   SIZE(nmesh)            ) \
  X ( vector, Matrix3Xi,        mesh,  face_texcoord, SIZE(nmesh)            ) \
  X ( vector, VectorXi,         mesh,  face_matidx,   SIZE(nmesh)            ) \
                                                                               \
  X ( vector, int,              cvx,   nameid,        SIZE(ncvx)             ) \
  X ( vector, int,              cvx,   nvert,         SIZE(ncvx)             ) \
  X ( vector, Matrix3Xr,        cvx,   vert,          SIZE(ncvx)             ) \
  X ( vector, uint8_t,          cvx,   flag,          SIZE(ncvx)             ) \
  X ( vector, real_t,           cvx,   a_thres,       SIZE(ncvx)             ) \
  X ( vector, vector<Matrix3r>, cvx,   vert_outer,    SIZE(ncvx)             ) \
                                                                               \
  X ( vector, int,              cvx2d, nameid,        SIZE(ncvx2d)           ) \
  X ( vector, int,              cvx2d, nvert,         SIZE(ncvx2d)           ) \
  X ( vector, Matrix2Xr,        cvx2d, vert,          SIZE(ncvx2d)           ) \
  X ( vector, uint8_t,          cvx2d, flag,          SIZE(ncvx2d)           ) \
  X ( vector, real_t,           cvx2d, a_thres,       SIZE(ncvx2d)           ) \
  X ( vector, vector<Matrix2r>, cvx2d, vert_outer,    SIZE(ncvx2d)           ) \
                                                                               \
  X ( vector, int,              sdf,   type,          SIZE(nsdf)             ) \
  X ( vector, int,              sdf,   nameid,        SIZE(nsdf)             ) \
  X ( vector, int,              sdf,   nparam,        SIZE(nsdf)             ) \
  X ( vector, VectorXr,         sdf,   param,         SIZE(nsdf)             ) \
                                                                               \
  X ( vector, int,              mat,   nameid,        SIZE(nmat)             ) \
  X ( vector, int,              mat,   texid,         SIZE(nmat)             ) \
  X ( vector, Vector4f,         mat,   rgba,          SIZE(nmat)             ) \
  X ( vector, Vector3f,         mat,   emission,      SIZE(nmat)             ) \
  X ( vector, Vector3f,         mat,   specular,      SIZE(nmat)             ) \
  X ( vector, float,            mat,   shininess,     SIZE(nmat)             ) \
                                                                               \
  X ( vector, int,              tex,   nameid,        SIZE(ntex)             ) \
  X ( vector, int,              tex,   pathid,        SIZE(ntex)             ) \
  X ( vector, int,              tex,   width,         SIZE(ntex)             ) \
  X ( vector, int,              tex,   height,        SIZE(ntex)             ) \
  X ( vector, vector<byte_t>,   tex,   rgb,           SIZE(ntex)             ) \
                                                                               \
  X ( vector, int,              cam,   nameid,        SIZE(ncam)             ) \
  X ( vector, int,              cam,   bodyid,        SIZE(ncam)             ) \
  X ( vector, Vector3f,         cam,   pos,           SIZE(ncam)             ) \
  X ( vector, Quaternionf,      cam,   quat,          SIZE(ncam)             ) \
  X ( vector, float,            cam,   fovy,          SIZE(ncam)             ) \
  X ( vector, float,            cam,   near,          SIZE(ncam)             ) \
  X ( vector, float,            cam,   far,           SIZE(ncam)             ) \
  X ( vector, bool,             cam,   overlay,       SIZE(ncam)             ) \
  X ( vector, int,              cam,   width,         SIZE(ncam)             ) \
  X ( vector, int,              cam,   height,        SIZE(ncam)             ) \
                                                                               \
  X ( vector, light_e,          light, type,          SIZE(nlight)           ) \
  X ( vector, bool,             light, enabled,       SIZE(nlight)           ) \
  X ( vector, bool,             light, cast_shadow,   SIZE(nlight)           ) \
  X ( vector, Vector3f,         light, pos,           SIZE(nlight)           ) \
  X ( vector, Vector3f,         light, dir,           SIZE(nlight)           ) \
  X ( vector, Vector3f,         light, ambient,       SIZE(nlight)           ) \
  X ( vector, Vector3f,         light, diffuse,       SIZE(nlight)           ) \
  X ( vector, Vector3f,         light, specular,      SIZE(nlight)           ) \
  X ( vector, Vector3f,         light, attenuation,   SIZE(nlight)           ) \
  X ( vector, float,            light, cutoff,        SIZE(nlight)           ) \
  X ( vector, float,            light, exponent,      SIZE(nlight)           ) \
                                                                               \
  X ( vector, actuator_e,       act,   type,          SIZE(nu)               ) \
  X ( vector, int,              act,   jointid,       SIZE(nu)               ) \
  X ( vector, int,              act,   local_dof,     SIZE(nu)               ) \
  X ( vector, Vector3r,         act,   gain,          SIZE(nu)               ) \
  X ( packed, real_t,           act,   u0,            SIZE(nu)               )

// dynamic matrix bump-allocated fields of model_t::mesh_t
#define CRISP_MODEL_BUMP_MESH(X, SIZE)                                         \
  X ( vector, Matrix3Xf, mesh, vert,          3, SIZE(nvert)                 ) \
  X ( vector, Matrix3Xf, mesh, normal,        3, SIZE(nnormal)               ) \
  X ( vector, Matrix2Xf, mesh, texcoord,      2, SIZE(ntexcoord)             ) \
  X ( vector, Matrix3Xi, mesh, face_vert,     3, SIZE(nface)                 ) \
  X ( vector, Matrix3Xi, mesh, face_normal,   3, SIZE(nface)                 ) \
  X ( vector, Matrix3Xi, mesh, face_texcoord, 3, SIZE(nface)                 ) \
  X ( vector, VectorXi,  mesh, face_matidx,   SIZE(nface)                    )

// bump-allocated fields of data_t
#define CRISP_DATA_BUMP(X, SIZE)                                               \
  X ( packed, real_t,        state, q,      SIZE(nq)                         ) \
  X ( packed, real_t,        state, v,      SIZE(nv)                         ) \
  X ( packed, real_t,        state, vhat,   SIZE(nv)                         ) \
                                                                               \
  X ( vector, Vector3r,      body,  pos,    SIZE(nbody)                      ) \
  X ( vector, Matrix3r,      body,  rot,    SIZE(nbody)                      ) \
  X ( vector, Quaternionr,   body,  quat,   SIZE(nbody)                      ) \
  X ( vector, Vector3r,      body,  ipos,   SIZE(nbody)                      ) \
  X ( vector, Matrix3r,      body,  irot,   SIZE(nbody)                      ) \
  X ( vector, Vector6r,      body,  v,      SIZE(nbody)                      ) \
  X ( vector, Vector6r,      body,  vhat,   SIZE(nbody)                      ) \
  X ( vector, Vector6r,      body,  cori,   SIZE(nbody)                      ) \
                                                                               \
  X ( vector, Vector6r,      dof,   screw,  SIZE(nv)                         ) \
  X ( packed, real_t,        dof,   f_bias, SIZE(nv)                         ) \
  X ( packed, real_t,        dof,   f_con,  SIZE(nv)                         ) \
                                                                               \
  X ( packed, real_t,        fric,  lambda, SIZE(nfric)                      ) \
                                                                               \
  X ( vector, real_t,        lim,   upper,  SIZE(nlim)                       ) \
  X ( vector, real_t,        lim,   lower,  SIZE(nlim)                       ) \
  X ( packed, real_t,        lim,   lambda, SIZE(nlim)                       ) \
                                                                               \
  X ( vector, Vector3r,      geom,  pos,    SIZE(ngeom)                      ) \
  X ( vector, Matrix3r,      geom,  rot,    SIZE(ngeom)                      ) \
  X ( vector, Vector6r,      geom,  aabb,   SIZE(ngeom)                      ) \
                                                                               \
  X ( vector, Vector6r,      dyn,   sp_f,   SIZE(nbody)                      ) \
  X ( vector, Vector10r,     dyn,   sp_I,   SIZE(nbody)                      ) \
  X ( vector, Vector6r,      dyn,   rne_f,  SIZE(nbody)                      ) \
  X ( vector, Vector10r,     dyn,   crb_I,  SIZE(nbody)                      ) \
  X ( unique, SparseMatrixr, dyn,   M,      SIZE(nv), SIZE(nv), SIZE(nM)     ) \
  X ( unique, SparseMatrixr, dyn,   M_UD,   SIZE(nv), SIZE(nv), SIZE(nM)     ) \
  X ( packed, real_t,        dyn,   M_Dinv, SIZE(nv)                         ) \
                                                                               \
  X ( packed, real_t,        act,   u,      SIZE(nu)                         ) \
  X ( packed, real_t,        act,   udot,   SIZE(nu)                         ) \
  X ( packed, real_t,        act,   f,      SIZE(nv)                         )

// stack-allocated fields of data_t in dense jacobian mode
#define CRISP_DATA_STACK_DENSE(X, M_SIZE, D_SIZE)                              \
  X ( vector, broad_pair_t,   con,   pair,    D_SIZE(nbroad)                 ) \
  X ( vector, con_feature_t,  con,   feature, D_SIZE(ncon)                   ) \
  X ( vector, int,            con,   card,    D_SIZE(ncon)                   ) \
  X ( vector, real_t,         con,   mu,      D_SIZE(ncon)                   ) \
  X ( vector, real_t,         con,   R,       D_SIZE(ncon)                   ) \
  X ( vector, Matrix3r,       con,   rot,     D_SIZE(ncon)                   ) \
  X ( packed, Vector3r,       con,   e,       D_SIZE(ncon)                   ) \
  X ( packed, Vector3r,       con,   lambda,  D_SIZE(ncon)                   ) \
  X ( unique, MatrixXr,       con,   J_d,     D_SIZE(nc), M_SIZE(nv)         )

// stack-allocated common fields of data_t::pgs_t
#define CRISP_DATA_STACK_PGS(X, M_SIZE, D_SIZE)                                \
  X ( packed, real_t, pgs, b,    M_SIZE(nv)                                  ) \
  X ( packed, real_t, pgs, Dinv, 3 * D_SIZE(ncon) + M_SIZE(nlim) +             \
                                 M_SIZE(nfric)                               )

// stack-allocated common fields of data_t::canal_t
#define CRISP_DATA_STACK_CANAL(X, M_SIZE, D_SIZE)                              \
  X ( packed, Vector3r, canal, e,          D_SIZE(ncon)                      ) \
  X ( packed, Vector3r, canal, z,          D_SIZE(ncon)                      ) \
  X ( packed, Vector3r, canal, u,          D_SIZE(ncon)                      ) \
  X ( packed, Vector3r, canal, u_old,      D_SIZE(ncon)                      ) \
  X ( unique, VectorXr, canal, grad,       M_SIZE(nv)                        ) \
  X ( packed, real_t,   canal, u_lim,      M_SIZE(nlim)                      ) \
  X ( packed, real_t,   canal, u_lim_old,  M_SIZE(nlim)                      ) \
  X ( packed, real_t,   canal, u_fric,     M_SIZE(nfric)                     ) \
  X ( packed, real_t,   canal, u_fric_old, M_SIZE(nfric)                     )

// stack-allocated common fields of data_t::admm_t
#define CRISP_DATA_STACK_SUBADMM(X, M_SIZE, D_SIZE)                            \
  X ( packed, Vector3r,      admm,   z,          D_SIZE(nc) / 3              ) \
  X ( packed, Vector3r,      admm,   u,          D_SIZE(nc) / 3              ) \
  X ( packed, Vector3r,      admm,   u_old,      D_SIZE(nc) / 3              ) \
  X ( packed, real_t,        admm,   u_lim,      M_SIZE(nlim)                ) \
  X ( packed, real_t,        admm,   u_lim_old,  M_SIZE(nlim)                ) \
  X ( packed, real_t,        admm,   z_lim,      M_SIZE(nlim)                ) \
  X ( packed, real_t,        admm,   u_fric,     M_SIZE(nfric)               ) \
  X ( packed, real_t,        admm,   u_fric_old, M_SIZE(nfric)               ) \
  X ( packed, real_t,        admm,   z_fric,     M_SIZE(nfric)               ) \
  X ( unique, SparseMatrixr, admm,   M_orig,     M_SIZE(nv), M_SIZE(nv),       \
                                                 M_SIZE(nM)                  ) \
  X ( vector, Vector10r,     admm,   sp_I_add,   M_SIZE(nbody)               )

#define CRISP_M_SIZE(NX) m.size.NX
#define CRISP_D_SIZE(NX) d.size.NX
