/*
    This file is part of Mitsuba, a physically based rendering system.

    Copyright (c) 2007-2012 by Wenzel Jakob and others.

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

#include <mitsuba/core/lock.h>
#include <mitsuba/core/fresolver.h>
#if defined(MTS_OPENMP)
# include <omp.h>
#endif

#include <boost/thread/thread.hpp>

// Required for native thread functions
#if defined(__LINUX__)
# include <sys/prctl.h>
#elif defined(__OSX__)
# include <pthread.h>
#elif defined(__WINDOWS__)
# include <windows.h>
#endif

MTS_NAMESPACE_BEGIN


#if defined(_MSC_VER)
namespace {
// Helper function to set a native thread name. MSDN:
//   http://msdn.microsoft.com/en-us/library/xcb2z8hs.aspx

const DWORD MS_VC_EXCEPTION = 0x406D1388;

#pragma pack(push, 8)
struct THREADNAME_INFO {
	DWORD dwType;     // Must be 0x1000.
	LPCSTR szName;    // Pointer to name (in user addr space).
	DWORD dwThreadID; // Thread ID (-1=caller thread).
	DWORD dwFlags;    // Reserved for future use, must be zero.
};
#pragma pack(pop)

void SetThreadName(const char* threadName, DWORD dwThreadID = -1) {
	THREADNAME_INFO info;
	info.dwType     = 0x1000;
	info.szName     = threadName;
	info.dwThreadID = dwThreadID;
	info.dwFlags    = 0;

	__try {
		RaiseException( MS_VC_EXCEPTION, 0, sizeof(info)/sizeof(ULONG_PTR),
			(ULONG_PTR*)&info );
	} __except(EXCEPTION_EXECUTE_HANDLER) { }
}


} // namespace
#endif // _MSC_VER

/**
 * Internal Thread members
 */
struct Thread::ThreadPrivate {
	ref<Thread> parent;
	ref<Logger> logger;
	ref<FileResolver> fresolver;
	boost::mutex joinMutex;
	std::string name;
	bool running, joined;
	Thread::EThreadPriority priority;
	static ThreadLocal<Thread> *self;
	bool critical;
	boost::thread thread;

	ThreadPrivate(const std::string & name_) :
		name(name_), running(false), joined(false),
		priority(Thread::ENormalPriority), critical(false) { }
};

/**
 * Dummy class to associate a thread identity with the main thread
 */
class MainThread : public Thread {
public:
	MainThread() : Thread("main") { }

	virtual void run() {
		Log(EError, "The main thread is already running!");
	}

	MTS_DECLARE_CLASS()
protected:
	virtual ~MainThread() { }
};

class UnmanagedThread : public Thread {
public:
	UnmanagedThread(const std::string &name)
		: Thread(name) { }

	virtual void run() {
		Log(EError, "The unmanaged thread is already running!");
	}

	MTS_DECLARE_CLASS()
protected:
	virtual ~UnmanagedThread() { }
};


ThreadLocal<Thread> *Thread::ThreadPrivate::self = NULL;

Thread::Thread(const std::string &name)
 : d(new ThreadPrivate(name)) { }

Thread::EThreadPriority Thread::getPriority() const {
	return d->priority;
}

void Thread::setCritical(bool critical) {
	d->critical = critical;
}

bool Thread::getCritical() const {
	return d->critical;
}

int Thread::getID() {
#if defined(__WINDOWS__)
	return static_cast<int>(GetCurrentThreadId());
#elif defined(__OSX__)
	return static_cast<int>(pthread_mach_thread_np(pthread_self()));
#else
	return (int) pthread_self();
#endif
}

const std::string& Thread::getName() const {
	return d->name;
}

void Thread::setName(const std::string &name) {
	d->name = name;
}

Thread* Thread::getParent() {
	return d->parent;
}

const Thread* Thread::getParent() const {
	return d->parent.get();
}

void Thread::setLogger(Logger *logger) {
	d->logger = logger;
}

Logger* Thread::getLogger() {
	return d->logger;
}

void Thread::setFileResolver(FileResolver *fresolver) {
	d->fresolver = fresolver;
}

FileResolver* Thread::getFileResolver() {
	return d->fresolver;
}

Thread* Thread::getThread() {
	return ThreadPrivate::self->get();
}

bool Thread::isRunning() const {
	return d->running;
}

void Thread::start() {
	if (d->running)
		Log(EError, "Thread is already running!");
	if (!d->self)
		Log(EError, "Threading has not been initialized!");

	Log(EDebug, "Spawning thread \"%s\"", d->name.c_str());

	d->parent = Thread::getThread();

	/* Inherit the parent thread's logger if none was set */
	if (!d->logger)
		d->logger = d->parent->getLogger();

	/* Inherit the parent thread's file resolver if none was set */
	if (!d->fresolver)
		d->fresolver = d->parent->getFileResolver();

	d->running = true;
	d->joined = false;

	incRef();
	try {
		d->thread = boost::thread(&Thread::dispatch, this);
	} catch (boost::thread_resource_error &ex) {
		Log(EError, "Could not create thread!");
		throw ex;
	}
}

bool Thread::setPriority(EThreadPriority priority) {
	d->priority = priority;
	if (!d->running)
		return true;

#if defined(__LINUX__) || defined(__OSX__)
	Float factor;
	switch (priority) {
		case EIdlePriority: factor = 0.0f; break;
		case ELowestPriority: factor = 0.2f; break;
		case ELowPriority: factor = 0.4f; break;
		case EHighPriority: factor = 0.6f; break;
		case EHighestPriority: factor = 0.8f; break;
		case ERealtimePriority: factor = 1.0f; break;
		default: factor = 0.0f; break;
	}

	const pthread_t threadID = d->thread.native_handle();
	struct sched_param param;
	int policy;
	int retval = pthread_getschedparam(threadID, &policy, &param);
	if (retval) {
		Log(EWarn, "pthread_getschedparam(): %s!", strerror(retval));
		return false;
	}

	int min = sched_get_priority_min(policy);
	int max = sched_get_priority_max(policy);

	if (min == max) {
		Log(EWarn, "Could not adjust the thread priority -- valid range is zero!");
		return false;
	}
	param.sched_priority = (int) (min + (max-min)*factor);

	retval = pthread_setschedparam(threadID, policy, &param);
	if (retval) {
		Log(EWarn, "Could not adjust the thread priority to %i: %s!",
			param.sched_priority, strerror(retval));
		return false;
	}
#elif defined(__WINDOWS__)
	int win32Priority;
	switch (priority) {
		case EIdlePriority:
			win32Priority = THREAD_PRIORITY_IDLE;
			break;
		case ELowestPriority:
			win32Priority = THREAD_PRIORITY_LOWEST;
			break;
		case ELowPriority:
			win32Priority = THREAD_PRIORITY_BELOW_NORMAL;
			break;
		case EHighPriority:
			win32Priority = THREAD_PRIORITY_ABOVE_NORMAL;
			break;
		case EHighestPriority:
			win32Priority = THREAD_PRIORITY_HIGHEST;
			break;
		case ERealtimePriority:
			win32Priority = THREAD_PRIORITY_TIME_CRITICAL;
			break;
		default:
			win32Priority = THREAD_PRIORITY_NORMAL;
			break;
	}

	// If the frunction succeeds, the return value is nonzero
	const HANDLE handle = d->thread.native_handle();
	if (SetThreadPriority(handle, win32Priority) == 0) {
		Log(EWarn, "Could not adjust the thread priority to %i: %s!",
			win32Priority, lastErrorText().c_str());
		return false;
	}
#else
	Log(EWarn, "Thread priority not supported by boost::thread version yet");
#endif
	return true;
}

void Thread::dispatch(Thread *thread) {
	detail::initializeLocalTLS();

	Thread::ThreadPrivate::self->set(thread);

	if (thread->getPriority() != ENormalPriority)
		thread->setPriority(thread->getPriority());

	if (!thread->getName().empty()) {
		const std::string threadName = "Mitsuba: " + thread->getName();
#if defined(__LINUX__)
		// Disabled for now, since it is not yet widely available in glibc
		// pthread_setname_np(pthread_self(), threadName.c_str());

		prctl(PR_SET_NAME, threadName.c_str());
#elif defined(__OSX__)
		pthread_setname_np(threadName.c_str());
#elif defined(__WINDOWS__)
		SetThreadName(threadName.c_str());
#endif
	}

	try {
		thread->run();
	} catch (std::exception &e) {
		ELogLevel warnLogLevel = thread->getLogger()->getErrorLevel() == EError
			? EWarn : EInfo;
		Log(warnLogLevel, "Fatal error: uncaught exception: \"%s\"", e.what());
		if (thread->d->critical)
			_exit(-1);
	} catch (...) {
		ELogLevel warnLogLevel = thread->getLogger()->getErrorLevel() == EError
			? EWarn : EInfo;
		Log(warnLogLevel, "Fatal error - uncaught exception (unknown type)");
		if (thread->d->critical)
			_exit(-1);
	}

	thread->exit();
}


void Thread::join() {
	/* Only one call to join() at a time */
	boost::lock_guard<boost::mutex> guard(d->joinMutex);
	if (d->joined)
		return;
	try {
		d->thread.join();
	} catch (boost::thread_interrupted &ex) {
		Log(EError, "Thread::join() - the thread was interrupted");
		throw ex;
	}
	d->joined = true;
}

void Thread::detach() {
	d->thread.detach();
}

void Thread::sleep(unsigned int ms) {
	try {
		boost::this_thread::sleep(boost::posix_time::milliseconds(ms));
	} catch (boost::thread_interrupted &ex) {
		Log(EError, "Thread::sleep(ms) - interrupted!");
		throw ex;
	}
}

void Thread::yield() {
	boost::this_thread::yield();
}

void Thread::exit() {
	Log(EDebug, "Thread \"%s\" has finished", d->name.c_str());
	d->running = false;
	Assert(ThreadPrivate::self->get() == this);
	detail::destroyLocalTLS();
	decRef();
}

std::string Thread::toString() const {
	std::ostringstream oss;
	oss << "Thread[" << endl
		<< "  name = \"" << d->name << "\"," << endl
		<< "  running = " << d->running << "," << endl
		<< "  joined = " << d->joined << "," << endl
		<< "  priority = " << d->priority << "," << endl
		<< "  critical = " << d->critical << endl
		<< "]";
	return oss.str();
}

static std::vector<Thread *> __unmanagedThreads;
static boost::mutex __unmanagedMutex;

#if defined(MTS_OPENMP) && defined(__OSX__)
static int __omp_threadCount = 0;
static pthread_key_t __omp_key;
static bool __omp_key_created;

int mts_omp_get_max_threads() {
	/* This function exists to sidestep an annoying
	   implementation bug that causes crashes on OSX */
	return __omp_threadCount;
}

int mts_omp_get_thread_num() {
	return reinterpret_cast<int>(pthread_getspecific(__omp_key));
}
#endif

void Thread::staticInitialization() {
#if defined(__OSX__)
	__mts_autorelease_init();
#if defined(MTS_OPENMP)
	__omp_threadCount = omp_get_max_threads();
#endif
#endif
	detail::initializeGlobalTLS();
	detail::initializeLocalTLS();

	ThreadPrivate::self = new ThreadLocal<Thread>();
	Thread *mainThread = new MainThread();
	mainThread->d->running = true;
	mainThread->d->joined = false;
	mainThread->d->fresolver = new FileResolver();
	ThreadPrivate::self->set(mainThread);
}

Thread *Thread::registerUnmanagedThread(const std::string &name) {
	Thread *thread = getThread();
	if (!thread) {
		thread = new UnmanagedThread(name);
		thread->d->running = false;
		thread->d->joined = false;
		thread->incRef();
		ThreadPrivate::self->set(thread);

		boost::lock_guard<boost::mutex> guard(__unmanagedMutex);
		__unmanagedThreads.push_back((UnmanagedThread *) thread);
	}
	return thread;
}

void Thread::staticShutdown() {
	for (size_t i=0; i<__unmanagedThreads.size(); ++i)
		__unmanagedThreads[i]->decRef();
	__unmanagedThreads.clear();
	getThread()->d->running = false;
	detail::destroyLocalTLS();
	delete ThreadPrivate::self;
	ThreadPrivate::self = NULL;
	detail::destroyGlobalTLS();
#if defined(__OSX__)
	#if defined(MTS_OPENMP)
		if (__omp_key_created)
			pthread_key_delete(__omp_key);
	#endif
	__mts_autorelease_shutdown();
#endif
}

void Thread::initializeOpenMP(size_t threadCount) {
#if defined(MTS_OPENMP)
	ref<Logger> logger = Thread::getThread()->getLogger();
	ref<FileResolver> fResolver = Thread::getThread()->getFileResolver();

	#if defined(__OSX__)
		if (!__omp_key_created) {
			pthread_key_create(&__omp_key, NULL);
			__omp_key_created = true;
		}
		__omp_threadCount = threadCount;
	#endif

	if (omp_get_dynamic())
		omp_set_dynamic(0);

	omp_set_num_threads((int) threadCount);

	int counter = 0;

	#pragma omp parallel
	{
		#if defined(__OSX__)
			if (!pthread_getspecific(__omp_key))
				pthread_setspecific(__omp_key, reinterpret_cast<void *>(counter));
		#endif
		detail::initializeLocalTLS();
		Thread *thread = Thread::getThread();
		if (!thread) {
			#pragma omp critical
			{
				thread = new UnmanagedThread(
					formatString("omp%i", counter));
				counter++;
			}
			const std::string threadName = "Mitsuba: " + thread->getName();

			#if defined(__LINUX__)
				prctl(PR_SET_NAME, threadName.c_str());
			#elif defined(__OSX__)
				pthread_setname_np(threadName.c_str());
			#elif defined(__WINDOWS__)
				SetThreadName(threadName.c_str());
			#endif

			thread->d->running = false;
			thread->d->joined = false;
			thread->d->fresolver = fResolver;
			thread->d->logger = logger;
			thread->incRef();
			ThreadPrivate::self->set(thread);

			#pragma omp critical
				__unmanagedThreads.push_back((UnmanagedThread *) thread);
		}
	}
#else
	if (Thread::getThread()->getLogger() != NULL)
		SLog(EWarn, "Mitsuba was compiled without OpenMP support.");
#endif
}

Thread::~Thread() {
	if (d->running)
		Log(EWarn, "Destructor called while thread '%s' was still running", d->name.c_str());
}

MTS_IMPLEMENT_CLASS(Thread, true, Object)
MTS_IMPLEMENT_CLASS(MainThread, false, Thread)
MTS_IMPLEMENT_CLASS(UnmanagedThread, false, Thread)
MTS_NAMESPACE_END
