/*
 * Copyright © 2021, Keith Packard <keithp@keithp.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "picolibc-hooks.h"

#ifdef CONFIG_MULTITHREADING

/*
 * Pick the picolibc retargetable-lock model that matches the active picolibc
 * source. The pinned module picolibc uses the old
 * `_LOCK_T = void *` model; the toolchain picolibc (SDK 0.17.x, selected when
 * full libc is required -- e.g. the TFLM/Ethos-U NPU build) uses the newer
 * `struct __lock` model. Upstream Zephyr moved to `struct __lock` wholesale,
 * which would break the default (module) build, so condition on the source.
 * A future SDK picolibc that ships this glue upstream would make the
 * conditional unnecessary.
 */
#ifdef CONFIG_PICOLIBC_USE_MODULE
/* Module picolibc -- old `_LOCK_T = void *` retarget-lock model. */
#define _LOCK_T void *
K_MUTEX_DEFINE(__lock___libc_recursive_mutex);

#ifdef CONFIG_USERSPACE
/* Grant public access to picolibc lock after boot */
static int picolibc_locks_prepare(void)
{

	/* Initialise recursive locks */
	k_object_access_all_grant(&__lock___libc_recursive_mutex);

	return 0;
}

SYS_INIT(picolibc_locks_prepare, POST_KERNEL,
	 CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
#endif /* CONFIG_USERSPACE */

/* Create a new dynamic recursive lock */
void __retarget_lock_init_recursive(_LOCK_T *lock)
{
	__ASSERT_NO_MSG(lock != NULL);

	/* Allocate mutex object */
#ifndef CONFIG_USERSPACE
	*lock = malloc(sizeof(struct k_mutex));
#else
	*lock = k_object_alloc(K_OBJ_MUTEX);
#endif /* !CONFIG_USERSPACE */
	__ASSERT(*lock != NULL, "recursive lock allocation failed");

	k_mutex_init((struct k_mutex *)*lock);
}

/* Create a new dynamic non-recursive lock */
void __retarget_lock_init(_LOCK_T *lock)
{
	__retarget_lock_init_recursive(lock);
}

/* Close dynamic recursive lock */
void __retarget_lock_close_recursive(_LOCK_T lock)
{
	__ASSERT_NO_MSG(lock != NULL);
#ifndef CONFIG_USERSPACE
	free(lock);
#else
	k_object_release(lock);
#endif /* !CONFIG_USERSPACE */
}

/* Close dynamic non-recursive lock */
void __retarget_lock_close(_LOCK_T lock)
{
	__retarget_lock_close_recursive(lock);
}

/* Acquire recursive lock */
void __retarget_lock_acquire_recursive(_LOCK_T lock)
{
	__ASSERT_NO_MSG(lock != NULL);
	k_mutex_lock((struct k_mutex *)lock, K_FOREVER);
}

/* Acquire non-recursive lock */
void __retarget_lock_acquire(_LOCK_T lock)
{
	__retarget_lock_acquire_recursive(lock);
}

/* Try acquiring recursive lock */
int __retarget_lock_try_acquire_recursive(_LOCK_T lock)
{
	__ASSERT_NO_MSG(lock != NULL);
	return !k_mutex_lock((struct k_mutex *)lock, K_NO_WAIT);
}

/* Try acquiring non-recursive lock */
int __retarget_lock_try_acquire(_LOCK_T lock)
{
	return __retarget_lock_try_acquire_recursive(lock);
}

/* Release recursive lock */
void __retarget_lock_release_recursive(_LOCK_T lock)
{
	__ASSERT_NO_MSG(lock != NULL);
	k_mutex_unlock((struct k_mutex *)lock);
}

/* Release non-recursive lock */
void __retarget_lock_release(_LOCK_T lock)
{
	__retarget_lock_release_recursive(lock);
}

#else /* !CONFIG_PICOLIBC_USE_MODULE */
/* Toolchain picolibc -- `struct __lock` model (backported from upstream Zephyr
 * lib/libc/picolibc/locks.c).
 */
#include <sys/lock.h>

/* Define the picolibc lock type */
struct __lock {
	struct k_mutex m;
};

STRUCT_SECTION_ITERABLE_ALTERNATE(k_mutex, __lock,
				  __lock___libc_recursive_mutex) = {
	.m = Z_MUTEX_INITIALIZER(__lock___libc_recursive_mutex.m),
};

#ifdef CONFIG_USERSPACE
/* Grant public access to picolibc lock after boot */
static int picolibc_locks_prepare(void)
{

	/* Initialise recursive locks */
	k_object_access_all_grant(&__lock___libc_recursive_mutex);

	return 0;
}

SYS_INIT(picolibc_locks_prepare, POST_KERNEL,
	 CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
#endif /* CONFIG_USERSPACE */

/* Create a new dynamic recursive lock */
void __retarget_lock_init_recursive(_LOCK_T *lock)
{
	__ASSERT_NO_MSG(lock != NULL);

	/* Allocate mutex object */
#ifndef CONFIG_USERSPACE
	*lock = malloc(sizeof(struct __lock));
#else
	*lock = k_object_alloc(K_OBJ_MUTEX);
#endif /* !CONFIG_USERSPACE */
	__ASSERT(*lock != NULL, "recursive lock allocation failed");

	k_mutex_init(&(*lock)->m);
}

/* Create a new dynamic non-recursive lock */
void __retarget_lock_init(_LOCK_T *lock)
{
	__retarget_lock_init_recursive(lock);
}

/* Close dynamic recursive lock */
void __retarget_lock_close_recursive(_LOCK_T lock)
{
	__ASSERT_NO_MSG(lock != NULL);
#ifndef CONFIG_USERSPACE
	free(lock);
#else
	k_object_release(lock);
#endif /* !CONFIG_USERSPACE */
}

/* Close dynamic non-recursive lock */
void __retarget_lock_close(_LOCK_T lock)
{
	__retarget_lock_close_recursive(lock);
}

/* Acquire recursive lock */
void __retarget_lock_acquire_recursive(_LOCK_T lock)
{
	__ASSERT_NO_MSG(lock != NULL);
	k_mutex_lock(&lock->m, K_FOREVER);
}

/* Acquire non-recursive lock */
void __retarget_lock_acquire(_LOCK_T lock)
{
	__retarget_lock_acquire_recursive(lock);
}

/* Try acquiring recursive lock */
int __retarget_lock_try_acquire_recursive(_LOCK_T lock)
{
	__ASSERT_NO_MSG(lock != NULL);
	return !k_mutex_lock(&lock->m, K_NO_WAIT);
}

/* Try acquiring non-recursive lock */
int __retarget_lock_try_acquire(_LOCK_T lock)
{
	return __retarget_lock_try_acquire_recursive(lock);
}

/* Release recursive lock */
void __retarget_lock_release_recursive(_LOCK_T lock)
{
	__ASSERT_NO_MSG(lock != NULL);
	k_mutex_unlock(&lock->m);
}

/* Release non-recursive lock */
void __retarget_lock_release(_LOCK_T lock)
{
	__retarget_lock_release_recursive(lock);
}

#endif /* CONFIG_PICOLIBC_USE_MODULE */

#endif /* CONFIG_MULTITHREADING */
