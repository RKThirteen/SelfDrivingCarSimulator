using System;
using System.Collections;
using System.Diagnostics;
using UnityEngine;

namespace Barmetler.RoadSystem.Util
{
    /// <summary>
    /// Updates a value when you call the Update function, but asynchronously.
    /// <para>This will make sure that the update was executed after the last time it was called, but only as often as necessary.</para>
    /// </summary>
    /// <typeparam name="T"></typeparam>
    public class AsyncUpdater<T>
    {
        private T _data;
        private readonly Func<T> _updater;

        private readonly MonoBehaviour _mb;
        private readonly object _dispatcherLock = new object();
        private readonly object _dataLock = new object();
        private bool _coroutineRunning;
        private bool _updateQueued;
        private readonly float _interval;
        private readonly Stopwatch _sw = new Stopwatch();

        public AsyncUpdater(MonoBehaviour mb, Func<T> updater, T initialData, float interval = 0)
        {
            _mb = mb;
            _updater = updater;
            _interval = interval;
            _data = initialData;
        }

        public AsyncUpdater(MonoBehaviour mb, Func<T> updater)
        {
            _mb = mb;
            _updater = updater;
        }

        /// <summary>
        /// Will make sure that the updater is called at some point in the future.
        /// </summary>
        public void Update()
        {
            _updateQueued = true;
            MaybeDispatchCoroutine();
        }

        /// <summary>
        /// Get current Data.
        /// </summary>
        public T GetData()
        {
            T d;
            lock (_dataLock)
                d = _data;
            return d;
        }

        private void MaybeDispatchCoroutine()
        {
            lock (_dispatcherLock)
            {
                if (!_coroutineRunning && _updateQueued)
                {
                    _updateQueued = false;
                    _coroutineRunning = true;
                    _mb.StartCoroutine(CallUpdater());
                }
            }
        }

        private IEnumerator CallUpdater()
        {
            _sw.Restart();
            var newData = _updater();
            _sw.Stop();
            var secondsToWait = (float)(_interval - _sw.ElapsedMilliseconds / 1e6);
            if (secondsToWait > 0)
                yield return new WaitForSeconds(secondsToWait);

            lock (_dataLock)
                _data = newData;

            _coroutineRunning = false;
            MaybeDispatchCoroutine();
            yield return null;
        }
    }
}
