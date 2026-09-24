/*
 * tc_dispatch.h  --  "dispatch slots": the mechanism that moves execution
 *                    from TC1 to TC2 while the program is running.
 *
 * This file does not depend on Pin, so it can be tested on its own
 * (see tests/test_dispatch.cpp).
 *
 * THE IDEA
 * --------
 * For every translated routine i we create two things:
 *
 *   slot[i]  : an 8-byte DATA word holding "where routine i runs right now".
 *              First it holds the TC1 address, later the TC2 address.
 *
 *   stub[i]  : a tiny piece of CODE that never changes:
 *                  jmp qword ptr [rip + (offset of slot[i])]
 *              i.e. "jump to whatever address is currently stored in slot[i]".
 *
 * The Pin probe at the start of the ORIGINAL routine jumps to stub[i]
 * (we pass stub[i] to RTN_ReplaceProbed, not the TC1 address). So:
 *
 *     original routine --probe--> stub[i] --reads slot[i]--> TC1 or TC2
 *
 * Switching routine i to TC2 is then ONE aligned 8-byte store into slot[i].
 * On x86-64 an aligned 8-byte store is atomic, so a thread that is just
 * executing the stub sees either the old address (TC1) or the new one (TC2),
 * never half of each. Both are valid code, so either result is safe.
 *
 * Why not just overwrite the probe's jmp instruction instead?
 *   - Rewriting instruction bytes while other threads may be executing them
 *     ("cross-modifying code") is tricky on x86. Here we never modify
 *     code at run time, we only modify data.
 *   - Pin only lets you place probes from the image-load callback, not later
 *     from our background thread.
 *
 * Memory layout (one mmap):
 *
 *     +---------------------------+  <- slots   (READ | WRITE)
 *     | slot[0] slot[1] ...       |
 *     +---------------------------+  <- stubs   (READ | EXEC)
 *     | stub[0] stub[1] ...       |     each stub = 8 bytes:
 *     +---------------------------+       FF 25 <disp32>   jmp [rip+disp32]
 *                                         CC CC            int3 padding
 *
 * Slots and stubs are on different pages, so writing a slot never touches a
 * code page. Being in the same mapping keeps them within +-2GB of each
 * other, which the rip-relative jmp needs.
 */
#ifndef TC_DISPATCH_H
#define TC_DISPATCH_H

#include <stdint.h>
#include <string.h>
#include <sys/mman.h>

struct dispatch_area_t {
    uint64_t      *slots;    // slots[i] = current entry address of routine i
    unsigned char *stubs;    // stub i starts at stubs + i * DISPATCH_STUB_SIZE
    size_t         count;    // number of routines
};

static const size_t DISPATCH_STUB_SIZE = 8;
static const size_t DISPATCH_PAGE_SIZE = 4096;

static inline size_t dispatch_round_up_to_page(size_t n)
{
    return (n + DISPATCH_PAGE_SIZE - 1) & ~(DISPATCH_PAGE_SIZE - 1);
}

/* Address of stub i: this is what you pass to RTN_ReplaceProbed(). */
static inline uintptr_t dispatch_stub_addr(const dispatch_area_t *d, size_t i)
{
    return (uintptr_t)(d->stubs + i * DISPATCH_STUB_SIZE);
}

/* Change where routine i runs. Safe to call while other threads run.
 * RELEASE ordering: everything we wrote before (the TC2 code) becomes
 * visible to other threads before they can see the new slot value. */
static inline void dispatch_set_target(dispatch_area_t *d, size_t i, uintptr_t target)
{
    __atomic_store_n(&d->slots[i], (uint64_t)target, __ATOMIC_RELEASE);
}

static inline uintptr_t dispatch_get_target(const dispatch_area_t *d, size_t i)
{
    return (uintptr_t)__atomic_load_n(&d->slots[i], __ATOMIC_ACQUIRE);
}

/*
 * Allocate slots + stubs for 'count' routines.
 * Every slot starts at 'initial_target' (can be 0 if you fill them in
 * later, but ALWAYS fill them in before the application runs).
 * Returns false on failure.
 */
static inline bool dispatch_create(dispatch_area_t *d, size_t count, uintptr_t initial_target)
{
    if (count == 0)
        return false;

    size_t slots_bytes = dispatch_round_up_to_page(count * sizeof(uint64_t));
    size_t stubs_bytes = dispatch_round_up_to_page(count * DISPATCH_STUB_SIZE);

    void *mem = mmap(NULL, slots_bytes + stubs_bytes, PROT_READ | PROT_WRITE,
                     MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (mem == MAP_FAILED)
        return false;

    d->slots = (uint64_t *)mem;
    d->stubs = (unsigned char *)mem + slots_bytes;
    d->count = count;

    for (size_t i = 0; i < count; i++) {
        d->slots[i] = (uint64_t)initial_target;

        // Encode:  jmp qword ptr [rip + disp32]   (opcode FF /4, modrm 0x25)
        // "rip" here means the address of the NEXT instruction = stub + 6.
        unsigned char *stub = d->stubs + i * DISPATCH_STUB_SIZE;
        int64_t disp = (int64_t)(uintptr_t)&d->slots[i] - (int64_t)((uintptr_t)stub + 6);
        int32_t disp32 = (int32_t)disp;   // always fits: same mapping

        stub[0] = 0xFF;
        stub[1] = 0x25;
        memcpy(&stub[2], &disp32, sizeof(disp32));
        stub[6] = 0xCC;   // int3 - never executed, just padding
        stub[7] = 0xCC;
    }

    // Stubs are finished: make them executable and read-only.
    if (mprotect(d->stubs, stubs_bytes, PROT_READ | PROT_EXEC) != 0) {
        munmap(mem, slots_bytes + stubs_bytes);
        return false;
    }
    return true;
}

#endif // TC_DISPATCH_H
