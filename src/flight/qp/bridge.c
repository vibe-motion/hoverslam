#include <stdlib.h>
#include <string.h>

#include "osqp.h"

typedef struct {
  OSQPSolver *solver;
  OSQPInt n;
  OSQPInt m;
  OSQPInt p_nnz;
  OSQPInt a_nnz;
} QPHandle;

enum {
  SET_VERBOSE = 1,
  SET_PROFILER_LEVEL,
  SET_WARM_STARTING,
  SET_SCALING,
  SET_POLISHING,
  SET_RHO,
  SET_RHO_IS_VEC,
  SET_SIGMA,
  SET_ALPHA,
  SET_CG_MAX_ITER,
  SET_CG_TOL_REDUCTION,
  SET_CG_TOL_FRACTION,
  SET_ADAPTIVE_RHO,
  SET_ADAPTIVE_RHO_INTERVAL,
  SET_ADAPTIVE_RHO_FRACTION,
  SET_ADAPTIVE_RHO_TOLERANCE,
  SET_MAX_ITER,
  SET_EPS_ABS,
  SET_EPS_REL,
  SET_EPS_PRIM_INF,
  SET_EPS_DUAL_INF,
  SET_SCALED_TERMINATION,
  SET_CHECK_TERMINATION,
  SET_CHECK_DUALGAP,
  SET_TIME_LIMIT,
  SET_DELTA,
  SET_POLISH_REFINE_ITER
};

static int apply_setting(OSQPSettings *s, OSQPInt key, OSQPFloat value) {
  switch (key) {
    case SET_VERBOSE: s->verbose = (OSQPInt)value; break;
    case SET_PROFILER_LEVEL: s->profiler_level = (OSQPInt)value; break;
    case SET_WARM_STARTING: s->warm_starting = (OSQPInt)value; break;
    case SET_SCALING: s->scaling = (OSQPInt)value; break;
    case SET_POLISHING: s->polishing = (OSQPInt)value; break;
    case SET_RHO: s->rho = value; break;
    case SET_RHO_IS_VEC: s->rho_is_vec = (OSQPInt)value; break;
    case SET_SIGMA: s->sigma = value; break;
    case SET_ALPHA: s->alpha = value; break;
    case SET_CG_MAX_ITER: s->cg_max_iter = (OSQPInt)value; break;
    case SET_CG_TOL_REDUCTION: s->cg_tol_reduction = (OSQPInt)value; break;
    case SET_CG_TOL_FRACTION: s->cg_tol_fraction = value; break;
    case SET_ADAPTIVE_RHO: s->adaptive_rho = (OSQPInt)value; break;
    case SET_ADAPTIVE_RHO_INTERVAL: s->adaptive_rho_interval = (OSQPInt)value; break;
    case SET_ADAPTIVE_RHO_FRACTION: s->adaptive_rho_fraction = value; break;
    case SET_ADAPTIVE_RHO_TOLERANCE: s->adaptive_rho_tolerance = value; break;
    case SET_MAX_ITER: s->max_iter = (OSQPInt)value; break;
    case SET_EPS_ABS: s->eps_abs = value; break;
    case SET_EPS_REL: s->eps_rel = value; break;
    case SET_EPS_PRIM_INF: s->eps_prim_inf = value; break;
    case SET_EPS_DUAL_INF: s->eps_dual_inf = value; break;
    case SET_SCALED_TERMINATION: s->scaled_termination = (OSQPInt)value; break;
    case SET_CHECK_TERMINATION: s->check_termination = (OSQPInt)value; break;
    case SET_CHECK_DUALGAP: s->check_dualgap = (OSQPInt)value; break;
    case SET_TIME_LIMIT: s->time_limit = value; break;
    case SET_DELTA: s->delta = value; break;
    case SET_POLISH_REFINE_ITER: s->polish_refine_iter = (OSQPInt)value; break;
    default: return -1;
  }
  return 0;
}

QPHandle *qp_create(OSQPInt n, OSQPInt m, OSQPInt p_nnz, OSQPInt a_nnz,
                    const OSQPInt *p, const OSQPInt *pi, const OSQPFloat *px,
                    const OSQPInt *a, const OSQPInt *ai, const OSQPFloat *ax,
                    const OSQPFloat *q, const OSQPFloat *l, const OSQPFloat *u,
                    const OSQPInt *setting_keys, const OSQPFloat *setting_values,
                    OSQPInt setting_count) {
  OSQPSettings settings;
  OSQPCscMatrix *P = NULL;
  OSQPCscMatrix *A = NULL;
  QPHandle *handle = NULL;
  OSQPInt k;
  OSQPInt err;

  if (n < 1 || m < 0 || p_nnz < 0 || a_nnz < 0 || !p || !pi || !px || !a ||
      !ai || !ax || !q || (m && (!l || !u)) || setting_count < 0) return NULL;

  osqp_set_default_settings(&settings);
  for (k = 0; k < setting_count; ++k) {
    if (apply_setting(&settings, setting_keys[k], setting_values[k])) return NULL;
  }

  P = OSQPCscMatrix_new(n, n, p_nnz, (OSQPFloat *)px, (OSQPInt *)pi, (OSQPInt *)p);
  A = OSQPCscMatrix_new(m, n, a_nnz, (OSQPFloat *)ax, (OSQPInt *)ai, (OSQPInt *)a);
  handle = (QPHandle *)calloc(1, sizeof(QPHandle));
  if (!P || !A || !handle) goto fail;

  handle->n = n;
  handle->m = m;
  handle->p_nnz = p_nnz;
  handle->a_nnz = a_nnz;
  err = osqp_setup(&handle->solver, P, q, A, l, u, m, n, &settings);
  OSQPCscMatrix_free(P);
  OSQPCscMatrix_free(A);
  if (err) {
    if (handle->solver) osqp_cleanup(handle->solver);
    free(handle);
    return NULL;
  }
  return handle;

fail:
  if (P) OSQPCscMatrix_free(P);
  if (A) OSQPCscMatrix_free(A);
  free(handle);
  return NULL;
}

int qp_solve(QPHandle *handle, int update_data,
             const OSQPFloat *px, const OSQPFloat *ax,
             const OSQPFloat *q, const OSQPFloat *l, const OSQPFloat *u,
             const OSQPFloat *warm_x, const OSQPFloat *warm_y,
             OSQPFloat *x_out) {
  OSQPInt err;
  if (!handle || !handle->solver || !px || !ax || !q || !x_out ||
      (handle->m && (!l || !u))) return -1;
  if (update_data) {
    err = osqp_update_data_mat(handle->solver, px, OSQP_NULL, handle->p_nnz,
                               ax, OSQP_NULL, handle->a_nnz);
    if (err) return (int)err;
    err = osqp_update_data_vec(handle->solver, q, l, u);
    if (err) return (int)err;
  }
  if (warm_x || warm_y) {
    err = osqp_warm_start(handle->solver, warm_x, warm_y);
    if (err) return (int)err;
  }
  err = osqp_solve(handle->solver);
  if (err) return (int)err;
  if (handle->solver->solution && handle->solver->solution->x)
    memcpy(x_out, handle->solver->solution->x, (size_t)handle->n * sizeof(OSQPFloat));
  return 0;
}

int qp_status(const QPHandle *handle) {
  return handle && handle->solver && handle->solver->info
             ? (int)handle->solver->info->status_val : OSQP_UNSOLVED;
}

int qp_iterations(const QPHandle *handle) {
  return handle && handle->solver && handle->solver->info
             ? (int)handle->solver->info->iter : 0;
}

double qp_primal_residual(const QPHandle *handle) {
  return handle && handle->solver && handle->solver->info
             ? handle->solver->info->prim_res : 0.0;
}

double qp_dual_residual(const QPHandle *handle) {
  return handle && handle->solver && handle->solver->info
             ? handle->solver->info->dual_res : 0.0;
}

void qp_destroy(QPHandle *handle) {
  if (!handle) return;
  if (handle->solver) osqp_cleanup(handle->solver);
  free(handle);
}
