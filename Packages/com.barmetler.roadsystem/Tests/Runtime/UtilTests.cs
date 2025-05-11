using System.Collections.Generic;
using System.Linq;
using Barmetler;
using Barmetler.RoadSystem.Util;
using NUnit.Framework;
using UnityEngine;
using RangeAttribute = NUnit.Framework.RangeAttribute;

namespace Tests.Runtime
{
	public class UtilTests
	{
		string[] randomWords = new[] {
			"pace"         ,
			"steel"        ,
			"abortion"     ,
			"disagreement" ,
			"bloodshed"    ,
			"driver"       ,
			"nun"          ,
			"pudding"      ,
			"divide"       ,
			"password"     ,
			"hold"         ,
			"block"        ,
			"dry"          ,
			"requirement"  ,
			"crisis"       ,
			"tumour"       ,
			"create"       ,
			"mention"      ,
			"cane"         ,
			"snow"         ,
			"knowledge"    ,
			"true"         ,
			"toll"         ,
			"category"     ,
			"dragon"       ,
			"equal"        ,
			"master"       ,
			"story"        ,
			"acute"        ,
			"withdrawal"   ,
			"essential"    ,
			"office"       ,
			"in"           ,
			"competence"   ,
			"transfer"     ,
			"sum"          ,
			"guitar"       ,
			"absorption"   ,
			"protection"   ,
			"instrument"   ,
			"can"          ,
			"recover"      ,
			"rumor"        ,
			"attitude"     ,
			"suspicion"    ,
			"budget"       ,
			"stretch"      ,
			"directory"    ,
			"systematic"   ,
			"steward"
		};

		(string[] concatenations, string initials) GetConcatenations(string[] words)
		{
			var delimiters = new[] { " ", "-", "_" };
			var ret = new List<string>();
			foreach (var delim in delimiters)
				ret.Add(string.Join(delim, words));
			ret.Add(string.Join("", words.Select(e => e.Length > 0 ? e.Substring(0, 1).ToUpper() + e.Substring(1) : e)));
			var initials = words.Where(e => e.Length > 0).Select(e => e.Substring(0, 1));
			return (ret.ToArray(), string.Join("", initials.Select(e => e.ToUpper())));
		}

		void AssertAreEqualEach(object a, IEnumerable<object> bs)
		{
			foreach (var b in bs)
			{
				Assert.AreEqual(a, b);
			}
		}

		#region StringUtility

		// Some Hard-coded examples
		[Test]
		public void StringUtility_GetInitials()
		{
			Assert.AreEqual("RS", StringUtility.GetInitials("RoadSystem"));
			Assert.AreEqual("RS", StringUtility.GetInitials("road_system"));
			Assert.AreEqual("RS", StringUtility.GetInitials("road system"));
			Assert.AreEqual("RS", StringUtility.GetInitials("road-system"));
		}

		[Test]
		public void StringUtility_GetInitials_PseudoRandom([Range(0, 100, 1)] int seed)
		{
			Random.InitState(seed);
			int wordCount = Random.Range(1, 6);
			var words = Enumerable.Range(0, wordCount).Select(_i => randomWords[Random.Range(0, randomWords.Length)]).ToArray();

			var (concats, initials) = GetConcatenations(words);
			AssertAreEqualEach(initials, concats.Select(StringUtility.GetInitials));
		}

		#endregion StringUtility

		#region TwoDimensionalArray

		[Test]
		public void TwoDimensionalArray_Construction([Range(1, 100, 7)] int width, [Range(1, 100, 11)] int height)
		{
			var arr = new TwoDimensionalArray<int>(width, height);
			Assert.AreEqual(width, arr.Width);
			Assert.AreEqual(height, arr.Height);
			Assert.AreEqual(width * height, arr.Length);
		}

		[Test]
		public void TwoDimensionalArray_Set_Get([Range(0, 50)] int seed)
		{
			Random.InitState(seed);

			int width = 128, height = 128;
			var arr = new TwoDimensionalArray<int>(width, height);
			var indices = Enumerable.Range(0, 500).Select(i => (x: Random.Range(0, width), y: Random.Range(0, height)));
			foreach (var index in indices)
			{
				int i = index.y * width + index.x;
				var v = new Vector2Int(index.x, index.y);
				arr[i] = 100;
				Assert.AreEqual(100, arr[i]);
				Assert.AreEqual(100, arr[index.x, index.y]);
				Assert.AreEqual(100, arr[v]);
				arr[i] = 0;
			}
		}

		/// <summary>
		/// Copy half square into the center of bigger square.
		/// at /4 of the dimensions of the array, the x-y-index of the source will be 0, and span to 1/2 at 3/4 of the array.
		/// 
		/// Example:
		/// -1 -1 -1 -1
		/// -1  1  2 -1
		/// -1  3  4 -1
		/// -1 -1 -1 -1
		/// </summary>
		[Test]
		public void TwoDimensionalArray_Copy_Into_Standard()
		{
			int width = 128, height = 128;
			var arr = new TwoDimensionalArray<float>(width, height);
			var src = new TwoDimensionalArray<float>(width / 2, height / 2);
			for (int i = 0; i < src.Length; ++i)
				src[i] = i + 1;
			src.CopyInto(-1, arr, new Vector2Int(width / 4, height / 4), Vector2Int.zero);
			for (int y = 0; y < height; ++y)
			{
				for (int x = 0; x < width; ++x)
				{
					int src_x = x - width / 4;
					int src_y = y - height / 4;
					if (src_x < 0 || src_x >= src.Width || src_y < 0 || src_y >= src.Height)
						Assert.AreEqual(-1, arr[x, y]);
					else
						Assert.AreEqual(src[src_x, src_y], arr[x, y]);
				}
			}
		}

		/// <summary>
		/// Copy half square into the center of a bigger square, but offset the cutout of the source by an eight.
		/// at /4 of the dimensions of the array, the x-y-index of the source will be /8, and span to 5/8 at 3/4 of the array.
		/// 
		/// Example:                 |  This is how src is offset before being inserted into arr:
		///                          |  You can see that the rows/columns 3...6 contain the values
		///                          |  you can see on the left. This visualization does not represent
		///                          |  the inner workings of the code, but rather show what it
		///                          |  effectively does. Keep in mind that 1/8 represents where
		///                          |  in src the copying should start, and not how much it is offset
		///                          |  from (0,0). I.e., if src_offset is (0,0), the copied section will have
		///                          |  a 1 in the top left, so the below representation would be moved down
		/// Result:                  |  and right by another step.
		///                          |  
		/// -1 -1 -1 -1 -1 -1 -1 -1  |  -- -- -- -- -- -- -- --
		/// -1 -1 -1 -1 -1 -1 -1 -1  |  --  1  2  3  4  5  6  7  8
		/// -1 -1 10 11 12 13 -1 -1  |  --  9 10 11 12 13 14 15 16
		/// -1 -1 18 19 20 21 -1 -1  |  -- 17 18 19 20 21 22 23 24
		/// -1 -1 26 27 28 29 -1 -1  |  -- 25 26 27 28 29 30 31 32
		/// -1 -1 34 35 36 37 -1 -1  |  -- 33 34 35 36 37 38 39 40
		/// -1 -1 -1 -1 -1 -1 -1 -1  |  -- 41 42 43 44 45 46 47 48
		/// -1 -1 -1 -1 -1 -1 -1 -1  |  -- 49 50 51 52 53 54 55 56
		///                                57 58 59 60 61 62 63 64
		/// </summary>
		[Test]
		public void TwoDimensionalArray_Copy_Into_Partial()
		{
			int width = 128, height = 128;
			var size = new Vector2Int(width, height);
			var arr = new TwoDimensionalArray<float>(width, height);
			var src = new TwoDimensionalArray<float>(width, height);
			for (int i = 0; i < src.Length; ++i)
				src[i] = i + 1;
			src.CopyInto(-1, arr, size / 4, size / 8, size / 2);
			for (int y = 0; y < height; ++y)
			{
				for (int x = 0; x < width; ++x)
				{
					if (x < width / 4 || x >= width * 3 / 4 || y < height / 4 || y >= height * 3 / 4)
						Assert.AreEqual(-1, arr[x, y]);
					else
						Assert.AreEqual(src[x - width / 8, y - height / 8], arr[x, y]);
				}
			}
		}

		#endregion TwoDimensionalArray

		#region DataCache

		[Test]
		public void DataCache_Test()
		{
			var cache = new DataCache<string>();
			Assert.IsFalse(cache.IsValid());
			cache.SetData("asd");
			Assert.IsTrue(cache.IsValid());
			Assert.AreEqual("asd", cache.GetData());
			cache.SetData("qwe");
			Assert.IsTrue(cache.IsValid());
			Assert.AreEqual("qwe", cache.GetData());
			cache.Invalidate();
			Assert.IsFalse(cache.IsValid());
		}

		#endregion DataCache

		#region ContextDataCache

		[Test]
		public void ContextDataCache_Test()
		{
			var cache = new ContextDataCache<string, string>();
			Assert.IsFalse(cache.IsValid("ctx1"));
			Assert.IsFalse(cache.IsValid("ctx2"));
			Assert.IsFalse(cache.IsValid("ctx3"));
			Assert.IsFalse(cache.IsValid("ctx4"));
			Assert.Throws<System.Exception>(() => cache.GetData("ctx1"));
			Assert.Throws<System.Exception>(() => cache.GetData("ctx2"));
			Assert.Throws<System.Exception>(() => cache.GetData("ctx3"));
			Assert.Throws<System.Exception>(() => cache.GetData("ctx4"));

			cache.SetData("data1", "ctx1");
			cache.SetData("data3", "ctx3");
			cache.SetData("data4", "ctx4");
			Assert.IsTrue(cache.IsValid("ctx1"));
			Assert.IsFalse(cache.IsValid("ctx2"));
			Assert.IsTrue(cache.IsValid("ctx3"));
			Assert.IsTrue(cache.IsValid("ctx4"));
			Assert.AreEqual("data1", cache.GetData("ctx1"));
			Assert.Throws<System.Exception>(() => cache.GetData("ctx2"));
			Assert.AreEqual("data3", cache.GetData("ctx3"));
			Assert.AreEqual("data4", cache.GetData("ctx4"));

			cache.SetData("data2", "ctx2");
			cache.SetData("data4.1", "ctx4");
			Assert.IsTrue(cache.IsValid("ctx1"));
			Assert.IsTrue(cache.IsValid("ctx2"));
			Assert.IsTrue(cache.IsValid("ctx3"));
			Assert.IsTrue(cache.IsValid("ctx4"));
			Assert.AreEqual("data1", cache.GetData("ctx1"));
			Assert.AreEqual("data2", cache.GetData("ctx2"));
			Assert.AreEqual("data3", cache.GetData("ctx3"));
			Assert.AreEqual("data4.1", cache.GetData("ctx4"));

			cache.Invalidate();
			Assert.IsFalse(cache.IsValid("ctx1"));
			Assert.IsFalse(cache.IsValid("ctx2"));
			Assert.IsFalse(cache.IsValid("ctx3"));
			Assert.IsFalse(cache.IsValid("ctx4"));
			Assert.Throws<System.Exception>(() => cache.GetData("ctx1"));
			Assert.Throws<System.Exception>(() => cache.GetData("ctx2"));
			Assert.Throws<System.Exception>(() => cache.GetData("ctx3"));
			Assert.Throws<System.Exception>(() => cache.GetData("ctx4"));
		}

		#endregion ContextDataCache
	}
}
